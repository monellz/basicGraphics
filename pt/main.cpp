#include "v3.hpp"
#include "utils.hpp"
#include "ray.hpp"
#include "scene.hpp"
#include "camera.hpp"
#include "tracer.hpp"
#include "render.hpp"

#include "ppm.hpp"

Scene scene;

int main(int argc, char*argv[])
{
    int w=atoi(argv[1]), h=atoi(argv[2]), samp=atoi(argv[4])/4;
    std::string fn(argv[3]);

    //Render* render = new PT(w,h,samp,&scene);
    Render* render = new PM(w,h,samp,&scene);
    //Render* render = new PPM(w,h,samp,&scene);
    render->rendering();
    render->generateIMG(fn);
    return 0;
}
