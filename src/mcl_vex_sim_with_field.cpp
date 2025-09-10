// mcl_vex_sim_with_field.cpp
// Single-file Monte Carlo Localization (AMCL-like) simulation with field image background
//
// Requirements:
//  - Put PBField.png in same folder
//  - Compile with: g++ -std=c++17 mcl_vex_sim_with_field.cpp -O2 -o mcl_sim -lm
//  - Run: ./mcl_sim
//
// Controls while running (terminal): w = forward, s = back, a = rotate left, d = rotate right, q = quit
//
// Notes:
//  - Map walls/obstacles are embedded below in "walls_init()" (units: mm)
//  - Long goals, upper middle goal, parking barriers are omitted per request
//  - Two distance sensors are simulated (offsets defined in SENSOR_* constants)
//  - This is a simulation only (no PROS). If you want a PROS-ready version, I can add it.

#include <bits/stdc++.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
using namespace std;

// ---------------- STB IMAGE (single-file) ----------------
// We include stb_image and stb_image_write to load PNG and write frames.
// Implementation macros are defined here so this single file works standalone.
//
// stb_image: https://github.com/nothings/stb
// stb_image_write: https://github.com/nothings/stb

#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_STATIC
#include <cstdlib>
extern "C" {
    // minimal include of stb_image implementation
    #include <stddef.h>
    #include <stdint.h>
    #define STBI_NO_STDIO
    #define STBI_ONLY_PNG
    #include "stb_image.h"
}

#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STB_IMAGE_WRITE_STATIC
extern "C" {
    #include "stb_image_write.h"
}

// Note: If your compiler can't find "stb_image.h" and "stb_image_write.h", you can
// download those headers into the same folder as this .cpp from the stb repo.
// But the versions included here assume the user has those files available in the include path.

// ---------------- end stb ----------------


// ---------- Simulation configuration ----------
static const int NUM_PARTICLES = 1200; // increase for accuracy; lower for speed (VEX Brain needs lower)
static const double SENSOR_STD = 25.0;         // mm
static const double ODOM_TRANS_STD = 3.5;      // mm
static const double ODOM_ROT_STD = (1.5 * M_PI/180.0); // radians
static const double MAX_SENSOR_RANGE = 2500.0; // mm
static const double ROBOT_RADIUS = 100.0;      // visualization radius (mm)
static const int IMAGE_OUT_W = 1200;
static const int IMAGE_OUT_H = 1200;
static const int STEPS = 2000;
static const double FIELD_SIZE = 3658.0; // mm (12 ft ~ 3658 mm)
// sensor geometry (front-left and front-right, in mm; angles in rad)
static const double S1_X = 80.0, S1_Y = 35.0, S1_ANG = 0.0;
static const double S2_X = 80.0, S2_Y = -35.0, S2_ANG = 0.0;
// motion commands
static const double CMD_FWD_STEP = 40.0;   // mm per keypress step
static const double CMD_ROT_STEP = 6.0 * M_PI/180.0; // rad per keypress

// ---------- Basic types ----------
struct Wall { double x1,y1,x2,y2; };
struct Particle { double x,y,th,w; };
struct Robot { double x,y,th; };

// ---------- Utility ----------
double angnorm(double a){
    while(a >= M_PI) a -= 2*M_PI;
    while(a < -M_PI) a += 2*M_PI;
    return a;
}
double clampd(double v,double a,double b){ if(v<a) return a; if(v>b) return b; return v; }

// ---------- Map initialization (per your request: exclude long goals, upper middle goal, parking barriers) ----------
// We'll include perimeter and central obstacles (approximate geometry).
vector<Wall> walls_init(){
    vector<Wall> w;
    // Perimeter (clockwise)
    w.push_back({0,0, FIELD_SIZE, 0});
    w.push_back({FIELD_SIZE, 0, FIELD_SIZE, FIELD_SIZE});
    w.push_back({FIELD_SIZE, FIELD_SIZE, 0, FIELD_SIZE});
    w.push_back({0, FIELD_SIZE, 0, 0});

    // Central "X" / cross (approximate). The real field has an X-logo; we model the physical central barriers.
    // Center coordinates:
    double cx = FIELD_SIZE/2.0, cy = FIELD_SIZE/2.0;
    double arm = 400.0; // half-length of cross arms
    // horizontal arm
    w.push_back({cx - arm, cy - 20, cx + arm, cy - 20});
    w.push_back({cx - arm, cy + 20, cx + arm, cy + 20});
    // vertical arm
    w.push_back({cx - 20, cy - arm, cx - 20, cy + arm});
    w.push_back({cx + 20, cy - arm, cx + 20, cy + arm});

    // Four small scoring pads near center (approx rectangles) approximated as 4 line segments each
    double padOff = 220.0;
    double padSize = 70.0;
    vector<pair<double,double>> padCenters = {
        {cx - padOff, cy - padOff},
        {cx + padOff, cy - padOff},
        {cx - padOff, cy + padOff},
        {cx + padOff, cy + padOff}
    };
    for(auto &pc : padCenters){
        double px = pc.first, py = pc.second;
        double hs = padSize/2.0;
        // rectangle (4 edges)
        w.push_back({px-hs, py-hs, px+hs, py-hs});
        w.push_back({px+hs, py-hs, px+hs, py+hs});
        w.push_back({px+hs, py+hs, px-hs, py+hs});
        w.push_back({px-hs, py+hs, px-hs, py-hs});
    }

    // Two small low walls on left/right near center (these are the small practice obstacles)
    w.push_back({cx - 800, cy - 450, cx - 650, cy - 450});
    w.push_back({cx + 650, cy + 450, cx + 800, cy + 450});

    // You can add more walls/obstacles here as needed (but we omit long goals and parking barriers)
    return w;
}

// ---------- Raycast to map walls (returns distance or max_range) ----------
double raycast_dist(const vector<Wall>& walls, double x, double y, double th, double max_range){
    double best = max_range;
    double dx = cos(th), dy = sin(th);
    for(const Wall &wd : walls){
        double x3 = wd.x1, y3 = wd.y1, x4 = wd.x2, y4 = wd.y2;
        double denom = (dx*(y3-y4) + dy*(x4-x3));
        if(fabs(denom) < 1e-9) continue;
        double t = ( (x3 - x) * (y3 - y4) + (y3 - y) * (x4 - x3) ) / denom;
        double u = ( (x3 - x) * dy - (y3 - y) * dx ) / denom;
        if(t >= 0 && t <= max_range && u >= 0.0 && u <= 1.0){
            if(t < best) best = t;
        }
    }
    return best;
}

// ---------- Systematic resample ----------
vector<Particle> resample_particles(const vector<Particle>& parts, std::mt19937 &rng){
    int M = parts.size();
    vector<double> cum(M);
    cum[0] = parts[0].w;
    for(int i=1;i<M;i++) cum[i] = cum[i-1] + parts[i].w;
    std::uniform_real_distribution<double> unif(0.0, 1.0/M);
    double r = unif(rng);
    vector<Particle> out; out.reserve(M);
    int idx = 0;
    for(int m=0;m<M;m++){
        double u = r + double(m)/M;
        while(u > cum[idx]) idx++;
        Particle np = parts[idx];
        np.w = 1.0 / M;
        out.push_back(np);
    }
    return out;
}

// ---------- Gaussian likelihood ----------
double gauss_prob(double diff, double sigma){
    double v = diff*diff;
    return exp(-0.5*v/(sigma*sigma)) / (sqrt(2*M_PI)*sigma);
}

// ---------- Weighted angle mean ----------
double weighted_angle_mean(const vector<Particle>& parts){
    double sx=0, sy=0;
    for(const auto &p: parts){
        sx += p.w * cos(p.th);
        sy += p.w * sin(p.th);
    }
    return atan2(sy, sx);
}

// ---------- Terminal raw mode for single-key input ----------
struct TermRaw {
    struct termios oldt;
    bool enabled = false;
    TermRaw(){ }
    void enable(){
        if(enabled) return;
        tcgetattr(STDIN_FILENO, &oldt);
        struct termios t = oldt;
        t.c_lflag &= ~(ICANON | ECHO); // raw mode
        t.c_cc[VMIN] = 0;
        t.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &t);
        enabled = true;
    }
    void disable(){
        if(!enabled) return;
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        enabled = false;
    }
    ~TermRaw(){ disable(); }
};

// ---------- Image helper: draw onto RGBA buffer ----------
struct Image {
    int w,h,channels;
    vector<unsigned char> data; // RGBA or RGB depending on channels
    Image():w(0),h(0),channels(3){}
    bool load_png(const string &filename){
        int comp;
        unsigned char *img = stbi_load(filename.c_str(), &w, &h, &comp, 4);
        if(!img) return false;
        channels = 4;
        data.assign(img, img + (w*h*4));
        stbi_image_free(img);
        return true;
    }
    // write PNG
    bool write_png(const string &filename) const {
        int stride = w * 4;
        return stbi_write_png(filename.c_str(), w, h, 4, data.data(), stride) != 0;
    }
    // putpixel (clamp)
    void putpx(int x,int y,unsigned char r,unsigned char g,unsigned char b,unsigned char a=255){
        if(x<0||x>=w||y<0||y>=h) return;
        int idx = (y*w + x)*4;
        // simple alpha over composite
        float af = a / 255.0f;
        data[idx+0] = (unsigned char)( (1-af)*data[idx+0] + af*r );
        data[idx+1] = (unsigned char)( (1-af)*data[idx+1] + af*g );
        data[idx+2] = (unsigned char)( (1-af)*data[idx+2] + af*b );
        data[idx+3] = 255;
    }
};

// ---------- draw helpers (world -> image coords) ----------
struct ViewTransform {
    double world_min_x, world_min_y, world_max_x, world_max_y;
    int img_w, img_h;
    double scale; // pixels per mm
    double offx, offy; // world coords that map to pixel (0,0)
    ViewTransform(){}
    void fit_to_world(double minx,double miny,double maxx,double maxy,int W,int H,double margin_px=20.0){
        world_min_x = minx; world_min_y = miny;
        world_max_x = maxx; world_max_y = maxy;
        img_w = W; img_h = H;
        double wx = maxx - minx, wy = maxy - miny;
        double sx = (W - 2*margin_px) / wx;
        double sy = (H - 2*margin_px) / wy;
        scale = min(sx, sy);
        // center
        double world_w_px = wx * scale, world_h_px = wy * scale;
        double left_margin = (W - world_w_px) / 2.0;
        double top_margin = (H - world_h_px) / 2.0;
        offx = minx - left_margin/scale;
        offy = miny - top_margin/scale;
    }
    inline int wx(double x) const { return int( (x - offx) * scale + 0.5 ); }
    inline int wy(double y) const { return int( img_h - 1 - ( (y - offy) * scale ) + 0.5 ); }
};

// ---------- Draw map, particles, robot, estimate onto image ----------
void render_frame(Image &bg, const ViewTransform &vt, const vector<Wall>& walls, const Robot &truth, const vector<Particle>& parts, double est_x, double est_y, double est_th){
    // start from background (already loaded)
    // draw walls (thin black lines)
    for(const Wall &wd : walls){
        int x0 = vt.wx(wd.x1), y0 = vt.wy(wd.y1);
        int x1 = vt.wx(wd.x2), y1 = vt.wy(wd.y2);
        // simple Bresenham
        int dx = abs(x1-x0), dy = abs(y1-y0);
        int sx = x0 < x1 ? 1 : -1;
        int sy = y0 < y1 ? 1 : -1;
        int err = dx - dy;
        int x=x0,y=y0;
        while(true){
            bg.putpx(x,y, 0,0,0, 255);
            if(x==x1 && y==y1) break;
            int e2 = 2*err;
            if(e2 > -dy){ err -= dy; x += sx; }
            if(e2 < dx){ err += dx; y += sy; }
        }
    }
    // draw particles (blueish, alpha by weight)
    for(const auto &p: parts){
        int px = vt.wx(p.x), py = vt.wy(p.y);
        double alpha = clampd(p.w * parts.size() * 1.5, 0.01, 0.9);
        unsigned char ri = (unsigned char)(20*(1-alpha) + 10*alpha);
        unsigned char gi = (unsigned char)(120*(1-alpha) + 40*alpha);
        unsigned char bi = (unsigned char)(255*(1-alpha) + 200*alpha);
        for(int dx=-1;dx<=1;dx++) for(int dy=-1;dy<=1;dy++) bg.putpx(px+dx, py+dy, ri,gi,bi, (unsigned char)(alpha*220));
    }
    // draw true robot (green)
    int tx = vt.wx(truth.x), ty = vt.wy(truth.y);
    for(int dx=-3;dx<=3;dx++) for(int dy=-3;dy<=3;dy++){
        if(dx*dx+dy*dy <= 9) bg.putpx(tx+dx, ty+dy, 0,200,0,255);
    }
    int hx = tx + int(cos(truth.th)*12.0), hy = ty - int(sin(truth.th)*12.0);
    for(int i=0;i<6;i++){
        int xi = tx + (hx-tx)*i/6;
        int yi = ty + (hy-ty)*i/6;
        bg.putpx(xi, yi, 0,255,0,255);
    }
    // draw estimated pose (red)
    int ex = vt.wx(est_x), ey = vt.wy(est_y);
    for(int dx=-3;dx<=3;dx++) for(int dy=-3;dy<=3;dy++) bg.putpx(ex+dx, ey+dy, 200,0,0,255);
    int ehx = ex + int(cos(est_th)*12.0), ehy = ey - int(sin(est_th)*12.0);
    for(int i=0;i<6;i++){
        int xi = ex + (ehx-ex)*i/6;
        int yi = ey + (ehy-ey)*i/6;
        bg.putpx(xi, yi, 255,0,0,255);
    }
}

// ---------- Main simulation ----------
int main(int argc, char** argv){
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    // Load field image
    Image bgImage;
    if(!bgImage.load_png("PBField.png")){
        cerr << "Failed to load PBField.png. Make sure it's in the same directory as the binary.\n";
        return 1;
    }
    // Convert loaded image to RGBA if not already (stb always returns 4 channels above)
    // We'll produce frames with the same size as bgImage, but to keep output sizes reasonable we rescale to IMAGE_OUT_W
    // Create a working image sized to IMAGE_OUT_W x IMAGE_OUT_H and draw background scaled into it.
    Image work;
    // rescale background to IMAGE_OUT_W x IMAGE_OUT_H using nearest-neighbor (simple)
    work.w = IMAGE_OUT_W; work.h = IMAGE_OUT_H; work.channels = 4;
    work.data.assign(work.w * work.h * 4, 255);
    // scale bgImage -> work
    for(int y=0;y<work.h;y++){
        for(int x=0;x<work.w;x++){
            int sx = int( (double)x / work.w * bgImage.w );
            int sy = int( (double)y / work.h * bgImage.h );
            if(sx<0) sx=0; if(sx>=bgImage.w) sx=bgImage.w-1;
            if(sy<0) sy=0; if(sy>=bgImage.h) sy=bgImage.h-1;
            int sidx = (sy*bgImage.w + sx)*4;
            int didx = (y*work.w + x)*4;
            work.data[didx+0] = bgImage.data[sidx+0];
            work.data[didx+1] = bgImage.data[sidx+1];
            work.data[didx+2] = bgImage.data[sidx+2];
            work.data[didx+3] = 255;
        }
    }

    // Map walls
    auto walls = walls_init();
    // calculate world bounding box (we know it's [0,FIELD_SIZE])
    double minx=0, miny=0, maxx=FIELD_SIZE, maxy=FIELD_SIZE;
    ViewTransform vt; vt.fit_to_world(minx, miny, maxx, maxy, work.w, work.h);

    // Initialize random generator
    std::random_device rd;
    std::mt19937 rng(rd());
    std::normal_distribution<double> odom_trans_noise(0.0, ODOM_TRANS_STD);
    std::normal_distribution<double> odom_rot_noise(0.0, ODOM_ROT_STD);
    std::normal_distribution<double> sensor_noise(0.0, SENSOR_STD);

    // Initialize robot truth near lower-left quarter so it isn't centered initially
    Robot truth{ FIELD_SIZE*0.3, FIELD_SIZE*0.4, 0.8 };

    // Initialize particles uniformly in field
    std::uniform_real_distribution<double> unifx(10.0, FIELD_SIZE - 10.0);
    std::uniform_real_distribution<double> unify(10.0, FIELD_SIZE - 10.0);
    std::uniform_real_distribution<double> unit(-M_PI, M_PI);
    vector<Particle> parts; parts.reserve(NUM_PARTICLES);
    for(int i=0;i<NUM_PARTICLES;i++){
        parts.push_back({ unifx(rng), unify(rng), unit(rng), 1.0 / NUM_PARTICLES });
    }

    TermRaw term; term.enable();
    bool quit = false;

    // Interactive command accumulation (keys held down)
    bool key_w=false, key_s=false, key_a=false, key_d=false;

    int step = 0;
    int frameCounter = 0;

    // main loop
    while(!quit && step < STEPS){
        // --- handle keyboard (non-blocking)
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(STDIN_FILENO, &readfds);
        struct timeval tv; tv.tv_sec = 0; tv.tv_usec = 0;
        int rv = select(STDIN_FILENO+1, &readfds, NULL, NULL, &tv);
        if(rv > 0 && FD_ISSET(STDIN_FILENO, &readfds)){
            char c;
            int n = read(STDIN_FILENO, &c, 1);
            if(n > 0){
                if(c == 'q') { quit = true; break; }
                else if(c == 'w') key_w = true;
                else if(c == 's') key_s = true;
                else if(c == 'a') key_a = true;
                else if(c == 'd') key_d = true;
                else if(c == 'W') key_w = false;
                else if(c == 'S') key_s = false;
                else if(c == '\n') { /* ignore */ }
                // Note: pressing uppercase toggles off in this simple scheme; you can adapt as needed.
            }
        }

        // Determine commanded motion from keys
        double cmd_fwd = 0.0;
        double cmd_rot = 0.0;
        if(key_w) cmd_fwd += CMD_FWD_STEP;
        if(key_s) cmd_fwd -= CMD_FWD_STEP;
        if(key_a) cmd_rot += CMD_ROT_STEP;
        if(key_d) cmd_rot -= CMD_ROT_STEP;

        // Apply to truth (ideal motion)
        double dx = cmd_fwd * cos(truth.th);
        double dy = cmd_fwd * sin(truth.th);
        truth.x += dx;
        truth.y += dy;
        truth.th = angnorm(truth.th + cmd_rot);
        // clamp inside field
        truth.x = clampd(truth.x, 5.0, FIELD_SIZE - 5.0);
        truth.y = clampd(truth.y, 5.0, FIELD_SIZE - 5.0);

        // Simulated sensor measurements from truth
        double s1wx = truth.x + cos(truth.th)*S1_X - sin(truth.th)*S1_Y;
        double s1wy = truth.y + sin(truth.th)*S1_X + cos(truth.th)*S1_Y;
        double s1ang = angnorm(truth.th + S1_ANG);
        double s2wx = truth.x + cos(truth.th)*S2_X - sin(truth.th)*S2_Y;
        double s2wy = truth.y + sin(truth.th)*S2_X + cos(truth.th)*S2_Y;
        double s2ang = angnorm(truth.th + S2_ANG);
        double meas1 = clampd(raycast_dist(walls, s1wx, s1wy, s1ang, MAX_SENSOR_RANGE) + sensor_noise(rng), 0.0, MAX_SENSOR_RANGE);
        double meas2 = clampd(raycast_dist(walls, s2wx, s2wy, s2ang, MAX_SENSOR_RANGE) + sensor_noise(rng), 0.0, MAX_SENSOR_RANGE);

        // Odometry reading (simulated) - we'll convert true dx,dy to forward distance + rotation and add noise
        double odo_forward = sqrt(dx*dx + dy*dy) + odom_trans_noise(rng);
        double odo_rot = cmd_rot + odom_rot_noise(rng);

        // Particle motion update
        for(auto &p: parts){
            double noisy_fwd = odo_forward + odom_trans_noise(rng);
            double noisy_rot = odo_rot + odom_rot_noise(rng);
            p.x += noisy_fwd * cos(p.th);
            p.y += noisy_fwd * sin(p.th);
            p.th = angnorm(p.th + noisy_rot);
            // keep inside bounds
            p.x = clampd(p.x, 0.0, FIELD_SIZE);
            p.y = clampd(p.y, 0.0, FIELD_SIZE);
        }

        // Sensor update: compute weight
        double total_w = 0.0;
        for(auto &p: parts){
            double ps1x = p.x + cos(p.th)*S1_X - sin(p.th)*S1_Y;
            double ps1y = p.y + sin(p.th)*S1_X + cos(p.th)*S1_Y;
            double ps1ang = angnorm(p.th + S1_ANG);
            double ps2x = p.x + cos(p.th)*S2_X - sin(p.th)*S2_Y;
            double ps2y = p.y + sin(p.th)*S2_X + cos(p.th)*S2_Y;
            double ps2ang = angnorm(p.th + S2_ANG);
            double exp1 = raycast_dist(walls, ps1x, ps1y, ps1ang, MAX_SENSOR_RANGE);
            double exp2 = raycast_dist(walls, ps2x, ps2y, ps2ang, MAX_SENSOR_RANGE);
            double p1 = gauss_prob(meas1 - exp1, SENSOR_STD);
            double p2 = gauss_prob(meas2 - exp2, SENSOR_STD);
            p.w = max(1e-14, p1 * p2);
            total_w += p.w;
        }

        // normalize
        if(total_w <= 0){
            double inv = 1.0 / parts.size();
            for(auto &p: parts) p.w = inv;
        } else {
            for(auto &p: parts) p.w /= total_w;
        }

        // Estimate pose
        double est_x = 0, est_y = 0;
        for(const auto &p: parts){ est_x += p.w * p.x; est_y += p.w * p.y; }
        double est_th = weighted_angle_mean(parts);

        // Resample if effective sample size low (optional). We'll resample every step for simplicity.
        parts = resample_particles(parts, rng);

        // Render frame
        Image frame = work; // copy background
        render_frame(frame, vt, walls, truth, parts, est_x, est_y, est_th);
        // save every N steps (or every step)
        char fname[256];
        snprintf(fname, sizeof(fname), "frame_%04d.png", frameCounter++);
        frame.write_png(fname);

        // Console status
        printf("step %4d | truth (%.1f,%.1f,%.1f°) | est (%.1f,%.1f,%.1f°) | meas (%.1f, %.1f)\n",
               step, truth.x, truth.y, truth.th*180.0/M_PI, est_x, est_y, est_th*180.0/M_PI, meas1, meas2);

        // small delay so files aren't generated too fast
        usleep(60000); // 60 ms per loop (~16 Hz)
        step++;
    }

    term.disable();
    cout << "Simulation ended. Frames saved as frame_####.png\n";
    return 0;
}
