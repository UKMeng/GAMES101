#include <algorithm>
#include <cassert>
#include <chrono>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    auto start = std::chrono::system_clock::now();
    if (primitives.empty())
        return;

    switch (splitMethod)
    {
        case SplitMethod::NAIVE:
            root = recursiveBuild(primitives);
            break;
        case SplitMethod::SAH:
            root = SAHBuild(primitives);
            break;
    }

    auto stop = std::chrono::system_clock::now();

    std::cout << "BVH Generation complete.\n";
    std::cout << "Time taken: " << std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count() << " milliseconds\n";

//    double diff = difftime(stop, start);
//    int hrs = (int)diff / 3600;
//    int mins = ((int)diff / 60) - (hrs * 60);
//    int secs = (int)diff - (hrs * 3600) - (mins * 60);
//
//    printf(
//        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
//        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
//    Bounds3 bounds;
//    for (int i = 0; i < objects.size(); ++i)
//        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }

        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    Intersection isect;
    if (node == nullptr) return isect;


    if (!node->bounds.IntersectP(ray, ray.direction_inv, {ray.direction.x > 0, ray.direction.y > 0, ray.direction.z > 0}))
    {
        return isect;
    }
    if (!node->left && !node->right) {
        isect = node->object->getIntersection(ray);
        return isect;
    }
    else {
        Intersection leftIsect = getIntersection(node->left, ray);
        Intersection rightIsect = getIntersection(node->right, ray);
        if (leftIsect.distance < rightIsect.distance)
            return leftIsect;
        else
            return rightIsect;
    }
}

BVHBuildNode *BVHAccel::SAHBuild(std::vector<Object *> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    struct Bucket
    {
        int startIndex;
        int endIndex;  // [start, end)
        Bounds3 bounds;
    };

    if (objects.size() == 1)
    {
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2)
    {
        node->left = SAHBuild(std::vector<Object*>{objects[0]});
        node->right = SAHBuild(std::vector<Object*>{objects[1]});
        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else
    {
        int bucketNum = 32; //std::min(16, (int)objects.size());
        int middle = 0;
        float leftS, rightS;
        float bestCost = FLT_MAX;

        Bounds3 bounds;
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
        {
            bounds = Union(bounds, objects[i]->getBounds());
            centroidBounds = Union(centroidBounds, objects[i]->getBounds().Centroid());
        }


        // init buckets
        std::vector<std::vector<Bucket>> totalBuckets(3, std::vector<Bucket>(bucketNum));
        for (int axis = 0; axis < 3; axis++)
        {
            for (int i = 0; i < bucketNum; ++i)
            {
                Vector3f bucketPMax = bounds.pMax;
                bucketPMax[axis] = bounds.pMin[axis] + (bucketPMax[axis] - bounds.pMin[axis]) * (i + 1) / bucketNum;
                totalBuckets[axis][i].bounds = Bounds3(bounds.pMin, bucketPMax);
            }
        }


        //int axis = centroidBounds.maxExtent();
        for (int axis = 0; axis < 3; axis++)
        {
            std::vector<Bucket>& buckets = totalBuckets[axis];

            switch (axis) {
                case 0:
                    std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                        return f1->getBounds().Centroid().x <
                               f2->getBounds().Centroid().x;
                    });
                    break;
                case 1:
                    std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                        return f1->getBounds().Centroid().y <
                               f2->getBounds().Centroid().y;
                    });
                    break;
                case 2:
                    std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                        return f1->getBounds().Centroid().z <
                               f2->getBounds().Centroid().z;
                    });
                    break;
            }

            // compute bucket
            int objectLeft = 0, objectRight = objects.size() - 1;

            for (int i = 0; i < bucketNum; ++i)
            {
                buckets[i].startIndex = objectLeft;
                while (objectLeft <= objectRight)
                {
                    int mid = objectLeft + (objectRight - objectLeft) / 2;

                    if (Bounds3::Inside(objects[mid]->getBounds().Centroid(), buckets[i].bounds))
                    {
                        objectLeft = mid + 1;
                    }
                    else
                    {
                        objectRight = mid - 1;
                    }
                }
                buckets[i].endIndex = objectLeft;
                objectRight = objects.size() - 1;
            }

            // calculate cost
            int leftObjectNum = 0;
            for (int i = 1; i < bucketNum; ++i)
            {
                leftObjectNum += buckets[i-1].endIndex - buckets[i-1].startIndex;

                if (leftObjectNum == 0 || leftObjectNum == objects.size()) continue;

                Bounds3 leftBounds, rightBounds;
                for (int j = 0; j < leftObjectNum; ++j)
                {
                    leftBounds = Union(leftBounds, objects[j]->getBounds());
                }
                for (int j = leftObjectNum; j < objects.size(); ++j)
                {
                    rightBounds = Union(rightBounds, objects[j]->getBounds());
                }

                double leftArea = leftBounds.SurfaceArea(), rightArea = rightBounds.SurfaceArea();
                float invDiv = 1.0 / (leftArea + rightArea);

                // float cost = invDiv * (i * leftObjectNum + (bucketNum - i) * (objects.size() - leftObjectNum));
                float cost = invDiv * (leftObjectNum * leftArea + (objects.size() - leftObjectNum) * rightArea);
                if (cost < bestCost)
                {
                    bestCost = cost;
                    middle = leftObjectNum;
                    leftS = leftArea;
                    rightS = rightArea;
                }
            }
        }

        std::vector<Object*> leftObjects, rightObjects;
        leftObjects.insert(leftObjects.end(), objects.begin(), objects.begin() + middle);
        rightObjects.insert(rightObjects.end(), objects.begin() + middle, objects.end());

        assert(objects.size() == (leftObjects.size() + rightObjects.size()));

        node->left = SAHBuild(leftObjects);
        node->right = SAHBuild(rightObjects);
        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}
