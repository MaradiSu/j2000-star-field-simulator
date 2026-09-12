// Terminal presentation; geometry remains in star_sim.cpp.
#include <chrono>
#include <thread>
#include <csignal>
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <conio.h>
#else
#include <unistd.h>
#include <sys/ioctl.h>
#include <termios.h>
#endif

volatile std::sig_atomic_t animationStop=0;
void stopAnimation(int) { animationStop=1; }
class TerminalScreen {
#ifdef _WIN32
    HANDLE original, screen;
#else
    termios original;
#endif
public:
    TerminalScreen() {
#ifdef _WIN32
        original=GetStdHandle(STD_OUTPUT_HANDLE);
        DWORD mode;
        if(!GetConsoleMode(original,&mode)) throw std::runtime_error("Animation requires an interactive terminal");
        screen=CreateConsoleScreenBuffer(GENERIC_READ|GENERIC_WRITE,FILE_SHARE_READ|FILE_SHARE_WRITE,NULL,CONSOLE_TEXTMODE_BUFFER,NULL);
        if(screen==INVALID_HANDLE_VALUE) throw std::runtime_error("Cannot create terminal screen");
        if(!SetConsoleActiveScreenBuffer(screen)) { CloseHandle(screen); throw std::runtime_error("Cannot activate terminal screen"); }
        CONSOLE_CURSOR_INFO cursor={1,FALSE}; SetConsoleCursorInfo(screen,&cursor);
#else
        if(!isatty(STDIN_FILENO)||!isatty(STDOUT_FILENO)||tcgetattr(STDIN_FILENO,&original)!=0)
            throw std::runtime_error("Animation requires an interactive terminal");
        termios mode=original; mode.c_lflag &= ~(ICANON|ECHO); mode.c_cc[VMIN]=0; mode.c_cc[VTIME]=0;
        if(tcsetattr(STDIN_FILENO,TCSANOW,&mode)!=0) throw std::runtime_error("Cannot configure terminal");
        std::cout << "\033[?1049h\033[?25l" << std::flush;
#endif
    }
    ~TerminalScreen() {
#ifdef _WIN32
        SetConsoleActiveScreenBuffer(original); CloseHandle(screen);
#else
        std::cout << "\033[?25h\033[?1049l" << std::flush;
        tcsetattr(STDIN_FILENO,TCSANOW,&original);
#endif
    }
    void size(int& w,int& h) {
#ifdef _WIN32
        CONSOLE_SCREEN_BUFFER_INFO s;
        if(!GetConsoleScreenBufferInfo(screen,&s)) throw std::runtime_error("Cannot read console size");
        w=s.srWindow.Right-s.srWindow.Left+1; h=s.srWindow.Bottom-s.srWindow.Top+1;
#else
        winsize s={};
        if(ioctl(STDOUT_FILENO,TIOCGWINSZ,&s)!=0) throw std::runtime_error("Cannot read terminal size");
        w=s.ws_col; h=s.ws_row;
#endif
        w=std::min(w,300); h=std::min(h,150);
        if(w<30||h<8) throw std::runtime_error("Resize terminal to at least 30 columns and 8 rows");
    }
    bool quit() {
#ifdef _WIN32
        if(_kbhit()) { int c=_getch(); return c=='q'||c=='Q'||c==27; }
#else
        char c; if(read(STDIN_FILENO,&c,1)==1) return c=='q'||c=='Q'||c==27;
#endif
        return animationStop!=0;
    }
    void show(const std::vector<std::string>& lines,int w,int h) {
#ifdef _WIN32
        std::vector<CHAR_INFO> cells(w*h);
        for(int y=0;y<h;++y) for(int x=0;x<w;++x) {
            cells[y*w+x].Char.AsciiChar=lines[y][x]; cells[y*w+x].Attributes=FOREGROUND_RED|FOREGROUND_GREEN|FOREGROUND_BLUE;
        }
        CONSOLE_SCREEN_BUFFER_INFO info; GetConsoleScreenBufferInfo(screen,&info);
        SMALL_RECT area={info.srWindow.Left,info.srWindow.Top,SHORT(info.srWindow.Left+w-1),SHORT(info.srWindow.Top+h-1)};
        COORD dims={SHORT(w),SHORT(h)}, origin={0,0};
        if(!WriteConsoleOutputA(screen,cells.data(),dims,origin,&area)) throw std::runtime_error("Console write failed");
#else
        std::ostringstream frame; frame << "\033[H";
        for(int y=0;y<h;++y) { frame << lines[y]; if(y<h-1) frame << "\r\n"; }
        std::cout << frame.str() << std::flush;
#endif
    }
};
// Fixed, deterministic synthetic sky over the whole celestial sphere.
std::vector<Star> animationCatalogue() {
    std::vector<Star> stars;
    const int n=5000;
    for(int i=0;i<n;++i) {
        double z=1-2*(i+0.5)/n, a=i*PI*(3-std::sqrt(5.0)), r=std::sqrt(1-z*z);
        stars.push_back({"S"+std::to_string(i),double(i%5),{r*std::cos(a),r*std::sin(a),z}});
    }
    return stars;
}
Quat rateAttitude(double rate,double t) {
    double half=rad(std::remainder(rate*t,360.0))/2;
    return {std::cos(half),0,-std::sin(half),0};
}
void animate(double duration,double rate) {
    if(duration<=0||duration>86400||std::abs(rate)>3600)
        throw std::runtime_error("Duration must be (0,86400] seconds; rate must be within +/-3600 deg/s");
    auto stars=animationCatalogue();
    animationStop=0;
    auto previous=std::signal(SIGINT,stopAnimation);
    try {
        TerminalScreen screen;
        const auto start=std::chrono::steady_clock::now();
        while(true) {
            auto now=std::chrono::steady_clock::now();
            double t=std::chrono::duration<double>(now-start).count();
            if(t>=duration||screen.quit()) break;
            int w,h; screen.size(w,h);
            Camera c={w,h-3,60,40,6,1,1,{1,0,0,0}};
            std::vector<std::string> rows(h,std::string(w,' '));
            Quat q=rateAttitude(rate,t);
            for(const Star& star:stars) {
                double u,v;
                if(project(rotate(q,star.inertial),c,u,v)) {
                    int x=int(std::floor(u+0.5)),y=int(std::floor(v+0.5))+2;
                    char glyph=star.mag<1?'@':star.mag<2?'*':star.mag<3?'+':'.';
                    if(rows[y][x]==' '||glyph=='@') rows[y][x]=glyph;
                }
            }
            std::ostringstream status;
            status << "STAR FIELD  " << std::fixed << std::setprecision(2) << t << "/" << duration << " s  rate " << rate << " deg/s";
            rows[0].replace(0,std::min(size_t(w-1),status.str().size()),status.str().substr(0,w-1));
            std::string hint="+Y rotation | +rate: stars left | Q/Esc: stop";
            rows[h-1].replace(0,std::min(size_t(w-1),hint.size()),hint.substr(0,w-1));
            screen.show(rows,w,h);
            // Wall-clock attitude avoids slowing the simulated rate when drawing is late.
            auto deadline=std::min(start+std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(duration)),now+std::chrono::milliseconds(33));
#ifdef _WIN32
            auto delay=std::chrono::duration_cast<std::chrono::milliseconds>(deadline-std::chrono::steady_clock::now()).count();
            if(delay>0) Sleep(static_cast<DWORD>(delay));
#else
            std::this_thread::sleep_until(deadline);
#endif
        }
    } catch(...) { std::signal(SIGINT,previous); throw; }
    std::signal(SIGINT,previous);
    std::cout << "Star simulation finished. Terminal restored.\n";
}

