add_executable(RayTracing main.cpp Object.hpp Vector.hpp Sphere.hpp global.hpp Triangle.hpp Scene.cpp Scene.hpp Light.hpp Renderer.cpp)
#target_compile_options(RayTracing PUBLIC -Wall -Wextra -pedantic -Wshadow -Wreturn-type -fsanitize=undefined)
#target_compile_features(RayTracing PUBLIC cxx_std_17)
#target_link_libraries(RayTracing PUBLIC -fsanitize=undefined)

target_compile_options(RayTracing PUBLIC
        $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -pedantic -Wshadow -Wreturn-type -fsanitize=undefined>
        $<$<CXX_COMPILER_ID:MSVC>:/W4 /permissive- /sdl /wd4244 /wd4267 /wd4996>
)

target_compile_features(RayTracing PUBLIC cxx_std_17)

target_link_libraries(RayTracing PUBLIC
        $<$<CXX_COMPILER_ID:GNU,Clang>:-fsanitize=undefined>
        $<$<CXX_COMPILER_ID:MSVC>:>
)