// Educational star-field simulator. C++11, standard library only.
// All CSV schemas and conventions are documented in README.md.
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

const double PI = 3.14159265358979323846;
double rad(double degrees) { return degrees * PI / 180.0; }
struct Vec { double x, y, z; };
struct Quat { double w, x, y, z; }; // Hamilton, scalar first
Quat unit(Quat q) {
    double n = std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    if (!std::isfinite(n) || n < 1e-12) throw std::runtime_error("Invalid quaternion");
    return {q.w/n, q.x/n, q.y/n, q.z/n};
}
Quat mul(Quat a, Quat b) {
    return {a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z,
            a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y,
            a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x,
            a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w};
}
Vec rotate(Quat q, Vec v) {
    Quat r = mul(mul(q, {0,v.x,v.y,v.z}), {q.w,-q.x,-q.y,-q.z});
    return {r.x,r.y,r.z};
}
Vec catalogueDirection(double ra, double dec) {
    double a=rad(ra), d=rad(dec);
    return {std::cos(d)*std::cos(a), std::cos(d)*std::sin(a), std::sin(d)};
}

typedef std::vector<std::vector<std::string> > Rows;
Rows readCsv(const std::string& path, const std::string& header, size_t columns) {
    std::ifstream f(path.c_str());
    if (!f) throw std::runtime_error("Cannot open " + path);
    Rows rows; std::string line; bool first=true;
    while (std::getline(f,line)) {
        if (!line.empty() && line.back()=='\r') line.pop_back();
        if (line.empty() || line[0]=='#') continue;
        if (first) {
            if (line!=header) throw std::runtime_error("Unexpected header in " + path);
            first=false; continue;
        }
        std::vector<std::string> row; std::stringstream ss(line); std::string cell;
        while (std::getline(ss,cell,',')) row.push_back(cell);
        if (row.size()!=columns) throw std::runtime_error("Wrong column count in " + path);
        rows.push_back(row);
    }
    if (first || rows.empty()) throw std::runtime_error("No records in " + path);
    return rows;
}
double number(const std::string& s) {
    size_t used=0; double n=std::stod(s,&used);
    if (used!=s.size() || !std::isfinite(n)) throw std::runtime_error("Invalid number: " + s);
    return n;
}
struct Camera {
    int width, height;
    double hfov, vfov, limit, sigma, gain;
    Quat qSB; // BODY components -> SENSOR components
};
Camera loadCamera(const std::string& path) {
    Rows r=readCsv(path,"width,height,hfov_deg,vfov_deg,mag_limit,psf_sigma_px,peak_dn_mag0,qSB_w,qSB_x,qSB_y,qSB_z",11);
    if(r.size()!=1) throw std::runtime_error("Camera requires exactly one row");
    double w=number(r[0][0]), h=number(r[0][1]);
    if(w<2 || h<2 || w>4096 || h>4096 || w!=std::floor(w) || h!=std::floor(h))
        throw std::runtime_error("Image dimensions must be integers in [2,4096]");
    Camera c={int(w),int(h),number(r[0][2]),number(r[0][3]),number(r[0][4]),
              number(r[0][5]),number(r[0][6]),
              unit({number(r[0][7]),number(r[0][8]),number(r[0][9]),number(r[0][10])})};
    if(c.hfov<=0 || c.hfov>=179 || c.vfov<=0 || c.vfov>=179 || c.sigma<0.3 || c.sigma>10 || c.gain<=0 || c.gain>65535)
        throw std::runtime_error("Invalid camera FOV, PSF or gain");
    return c;
}
struct Star { std::string id; double mag; Vec inertial; };
std::vector<Star> loadStars(const std::string& path) {
    Rows rows=readCsv(path,"id,ra_deg,dec_deg,mag",4); std::vector<Star> stars;
    for (const auto& r:rows) {
        double ra=number(r[1]), dec=number(r[2]), mag=number(r[3]);
        if(ra<0 || ra>=360 || dec < -90 || dec>90 || mag < -30 || mag>40)
            throw std::runtime_error("Catalogue value outside supported range");
        stars.push_back({r[0],mag,catalogueDirection(ra,dec)});
    }
    return stars;
}
struct Attitude { double t; Quat qBI; };
std::vector<Attitude> loadAttitudes(const std::string& path) {
    Rows rows=readCsv(path,"time_s,qBI_w,qBI_x,qBI_y,qBI_z",5); std::vector<Attitude> a;
    for(const auto& r:rows) {
        double t=number(r[0]);
        if(t<0 || (!a.empty() && t<=a.back().t)) throw std::runtime_error("Times must increase and be nonnegative");
        a.push_back({t,unit({number(r[1]),number(r[2]),number(r[3]),number(r[4])})});
    }
    return a;
}
// Right-handed sensor axes: +X image-right, +Y image-down, +Z boresight.
// Virtual upright pinhole plane; no physical focal-plane inversion applied.
bool project(Vec s, const Camera& c, double& u, double& v) {
    if(s.z<=0) return false;
    double fx=c.width/(2*std::tan(rad(c.hfov)/2));
    double fy=c.height/(2*std::tan(rad(c.vfov)/2));
    u=(c.width-1)/2.0 + fx*s.x/s.z;
    v=(c.height-1)/2.0 + fy*s.y/s.z;
    return u>=-0.5 && u<c.width-0.5 && v>=-0.5 && v<c.height-0.5;
}
void drawStar(std::vector<double>& pixels, const Camera& c, double u, double v, double mag) {
    double peak=c.gain*std::pow(10.0,-0.4*mag);
    int radius=int(std::ceil(4*c.sigma));
    int x0=std::max(0,int(std::floor(u))-radius), x1=std::min(c.width-1,int(std::ceil(u))+radius);
    int y0=std::max(0,int(std::floor(v))-radius), y1=std::min(c.height-1,int(std::ceil(v))+radius);
    for(int y=y0;y<=y1;++y) for(int x=x0;x<=x1;++x) {
        double dx=x-u, dy=y-v;
        pixels[y*c.width+x]+=peak*std::exp(-(dx*dx+dy*dy)/(2*c.sigma*c.sigma));
    }
}
void writePgm(const std::string& path, const std::vector<double>& pixels, const Camera& c) {
    std::ofstream f(path.c_str(),std::ios::binary);
    if(!f) throw std::runtime_error("Cannot write " + path);
    f << "P5\n" << c.width << " " << c.height << "\n65535\n";
    for(double p:pixels) {
        unsigned int dn=static_cast<unsigned int>(std::min(65535.0,std::max(0.0,p))+0.5);
        f.put(static_cast<char>((dn>>8)&255)); f.put(static_cast<char>(dn&255));
    }
    if(!f) throw std::runtime_error("Image write failed");
}
void require(bool b, const char* message) { if(!b) throw std::runtime_error(message); }
void selfTest() {
    Camera c={512,512,20,20,6,1,50000,{1,0,0,0}};
    double u=0,v=0;
    require(project({0,0,1},c,u,v) && std::abs(u-255.5)<1e-10 && std::abs(v-255.5)<1e-10,"Boresight test failed");
    require(!project({0,0,-1},c,u,v),"Rear hemisphere test failed");
    require(!project({std::tan(rad(11)),0,1},c,u,v),"FOV rejection failed");
    require(project({std::tan(rad(5)),0,1},c,u,v) && u>255.5,"X sign test failed");
    require(project({0,std::tan(rad(5)),1},c,u,v) && v>255.5,"Y sign test failed");
    Vec r=rotate(unit({std::cos(PI/4),0,std::sin(PI/4),0}),{1,0,0});
    require(std::abs(r.x)<1e-10 && std::abs(r.z+1)<1e-10,"Quaternion direction test failed");
    // Noncommuting mount and attitude: qBI turns +X into +Y, then qSB turns +Y into +Z.
    Quat bi=unit({1,0,0,1}), sb=unit({1,1,0,0});
    r=rotate(mul(sb,bi),{1,0,0});
    require(std::abs(r.z-1)<1e-10,"Mount composition failed");
    Vec pole=catalogueDirection(0,90);
    require(std::abs(pole.z-1)<1e-10,"Catalogue conversion failed");
    require(std::abs(std::pow(10.0,-0.4*5)-0.01)<1e-12,"Magnitude law failed");
    bool rejected=false; try { unit({0,0,0,0}); } catch(const std::exception&) { rejected=true; }
    require(rejected,"Zero quaternion accepted");
    std::cout << "All geometry/convention self-tests passed\n";
}
int main(int argc, char** argv) {
    try {
        if(argc==2 && std::string(argv[1])=="--self-test") { selfTest(); return 0; }
        if(argc!=5) {
            std::cerr << "Usage: star_sim catalogue.csv attitude.csv camera.csv output_prefix\n"
                      << "       star_sim --self-test\n"; return 1;
        }
        auto stars=loadStars(argv[1]); auto attitudes=loadAttitudes(argv[2]); Camera c=loadCamera(argv[3]);
        std::string prefix=argv[4];
        std::ofstream spots((prefix+"spots.csv").c_str()), commands((prefix+"mock_stos_commands.csv").c_str());
        if(!spots || !commands) throw std::runtime_error("Output directory must already exist");
        spots << std::setprecision(15) << "frame,time_s,id,u_px,v_px,mag,sensor_x,sensor_y,sensor_z\n";
        commands << std::setprecision(15) << "frame,time_s,qSI_w,qSI_x,qSI_y,qSI_z\n";
        for(size_t i=0;i<attitudes.size();++i) {
            const Attitude& a=attitudes[i];
            // qSI maps inertial components directly into sensor components.
            Quat qSI=unit(mul(c.qSB,a.qBI));
            commands << i << ',' << a.t << ',' << qSI.w << ',' << qSI.x << ',' << qSI.y << ',' << qSI.z << '\n';
            std::vector<double> pixels(c.width*c.height,0.0); int count=0;
            for(const Star& star:stars) {
                if(star.mag>c.limit) continue;
                Vec s=rotate(qSI,star.inertial); double u,v;
                if(!project(s,c,u,v)) continue;
                drawStar(pixels,c,u,v,star.mag); ++count;
                spots << i << ',' << a.t << ',' << star.id << ',' << u << ',' << v << ',' << star.mag
                      << ',' << s.x << ',' << s.y << ',' << s.z << '\n';
            }
            std::ostringstream name; name << prefix << "frame_" << std::setfill('0') << std::setw(4) << i << ".pgm";
            writePgm(name.str(),pixels,c);
            std::cout << "frame=" << i << " time=" << a.t << " s visible_stars=" << count << '\n';
        }
        if(!spots || !commands) throw std::runtime_error("CSV write failed");
        return 0;
    } catch(const std::exception& e) { std::cerr << "Error: " << e.what() << '\n'; return 1; }
}
