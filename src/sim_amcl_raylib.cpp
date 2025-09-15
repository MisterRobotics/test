// sim_amcl_raylib.cpp
// Adaptive Monte Carlo Localization simulation using raylib
// Controls: W forward, S back, A turn left, D turn right, Q quit
// Usage: ./sim_amcl <field_image.png> <map.txt>
#include <raylib.h>
#include <cmath>
#include <vector>
#include <random>
#include <string>
#include <fstream>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <chrono>
#include <iomanip>

// ----- CONFIG -----
static const double FIELD_SIZE_MM = 3658.0; // mm (12 ft)
static const int WIN_W = 1000;
static const int WIN_H = 1000;
static const double PIXELS_PER_MM_DEFAULT = (double)WIN_W / FIELD_SIZE_MM;

static const int MIN_PARTICLES = 200;
static const int MAX_PARTICLES = 400;
static const int INIT_PARTICLES = 300;

// Odometry noise (std dev)
static const double ODOM_TRANS_STD = 4.0; // mm
static const double ODOM_ROT_STD = (1.2 * M_PI/180.0); // rad

// Sensor model noise (std dev)
static const double SENSOR_STD = 30.0; // mm

// Resample threshold (effective sample size ratio)
static const double RESAMPLE_NEFF_RATIO = 0.5;

// Sensor offsets (mm) and relative angles (rad)
struct SensorSpec { double dx, dy, angle; };
static const SensorSpec SENSORS[3] = {
    { -45.0,  0.0,  M_PI },    // back sensor (behind center)
    {  0.0,  40.0,  M_PI/2 },  // left sensor
    {  0.0, -40.0, -M_PI/2 }   // right sensor
};
static const double MAX_SENSOR_RANGE = 2200.0; // mm

// Motion step magnitudes for keyboard control
static const double CMD_FWD = 40.0; // mm per step
static const double CMD_ROT = 6.0 * M_PI/180.0; // rad per step

// ----- TYPES -----
struct Wall { double x1,y1,x2,y2; };
struct Particle { double x,y,th; double w; };
struct Robot { double x,y,th; };

// ----- RNG -----
static std::mt19937 rng((unsigned)std::chrono::high_resolution_clock::now().time_since_epoch().count());

// ----- Utility -----
inline double angnorm(double a) {
    while(a >= M_PI) a -= 2*M_PI;
    while(a < -M_PI) a += 2*M_PI;
    return a;
}
inline double sqr(double x) { return x*x; }

// ----- Map loader -----
bool load_map(const std::string &path, std::vector<Wall> &walls) {
    walls.clear();
    std::ifstream in(path);
    if(!in) {
        std::cerr << "Failed to open map file: " << path << "\n";
        return false;
    }
    std::string line;
    while(std::getline(in, line)) {
        if(line.empty()) continue;
        std::istringstream ss(line);
        if(line[0] == '#') continue;
        double x1,y1,x2,y2;
        if(ss >> x1 >> y1 >> x2 >> y2) {
            walls.push_back({x1,y1,x2,y2});
        }
    }
    return true;
}

// ----- Raycast to walls (returns distance in mm, <= max_range) -----
double raycast(const std::vector<Wall> &walls, double x, double y, double th, double max_range) {
    double best = max_range;
    double dx = cos(th), dy = sin(th);
    for(const Wall &w : walls) {
        double x3 = w.x1, y3 = w.y1, x4 = w.x2, y4 = w.y2;
        double denom = (dx*(y3-y4) + dy*(x4-x3));
        if(std::abs(denom) < 1e-9) continue;
        double t = ((x3 - x)*(y3 - y4) + (y3 - y)*(x4 - x3)) / denom;
        double u = ((x3 - x)*dy - (y3 - y)*dx) / denom;
        if(t >= 0.0 && t <= max_range && u >= 0.0 && u <= 1.0) {
            if(t < best) best = t;
        }
    }
    return best;
}

// ----- Gaussian PDF (1D) -----
double gaussian_pdf(double diff, double sigma) {
    double v = diff*diff;
    double denom = sigma * sqrt(2.0*M_PI);
    return exp(-0.5 * v / (sigma*sigma)) / denom;
}

// ----- Particle initialization -----
// Positions uniform across field; headings within +/-5 degrees of truth
void init_particles(std::vector<Particle> &parts, int N, const Robot &truth, double field_min_x, double field_min_y, double field_max_x, double field_max_y) {
    std::uniform_real_distribution<double> ux(field_min_x+10.0, field_max_x-10.0);
    std::uniform_real_distribution<double> uy(field_min_y+10.0, field_max_y-10.0);
    std::uniform_real_distribution<double> uth(-5.0*M_PI/180.0, 5.0*M_PI/180.0);
    parts.clear();
    parts.reserve(N);
    for(int i=0;i<N;i++){
        double x = ux(rng);
        double y = uy(rng);
        double th = angnorm(truth.th + uth(rng));
        parts.push_back({x,y,th, 1.0/N});
    }
}

// ----- Effective sample size -----
double effective_sample_size(const std::vector<Particle> &parts){
    double s = 0.0;
    for(const auto &p: parts) s += p.w * p.w;
    if(s <= 0) return 0.0;
    return 1.0 / s;
}

// ----- Systematic resampling -----
std::vector<Particle> systematic_resample(const std::vector<Particle> &parts) {
    int M = parts.size();
    std::vector<double> cum(M);
    cum[0] = parts[0].w;
    for(int i=1;i<M;i++) cum[i] = cum[i-1] + parts[i].w;
    std::uniform_real_distribution<double> u(0.0, 1.0/M);
    double r = u(rng);
    std::vector<Particle> out; out.reserve(M);
    int idx = 0;
    for(int m=0;m<M;m++){
        double U = r + double(m)/M;
        while(U > cum[idx]) idx++;
        Particle np = parts[idx];
        np.w = 1.0 / M;
        out.push_back(np);
    }
    return out;
}

// ----- Weighted mean pose estimate -----
void estimate_pose(const std::vector<Particle> &parts, double &est_x, double &est_y, double &est_th) {
    est_x = 0.0; est_y = 0.0;
    double sx = 0.0, sy = 0.0;
    for(const auto &p: parts){
        est_x += p.w * p.x;
        est_y += p.w * p.y;
        sx += p.w * cos(p.th);
        sy += p.w * sin(p.th);
    }
    est_th = atan2(sy, sx);
}

// ----- Main -----
int main(int argc, char** argv) {
    std::string field_image = "Maps/PBField.png";
    std::string mapfile = "Maps/map.txt";
    if(argc >= 2) field_image = argv[1];
    if(argc >= 3) mapfile = argv[2];

    // Load map
    std::vector<Wall> walls;
    if(!load_map(mapfile, walls)) {
        std::cerr << "Map load failed. Create a map file with wall segments (x1 y1 x2 y2) in mm.\n";
        return 1;
    }

    // Init window
    InitWindow(WIN_W, WIN_H, "AMCL Raylib Simulator");
    SetTargetFPS(60);

    // Load texture
    Texture2D fieldTex = LoadTexture(field_image.c_str());
    // compute transform from world(mm) -> screen(px)
    double pixels_per_mm = PIXELS_PER_MM_DEFAULT;
    double scaleX = (double)WIN_W / FIELD_SIZE_MM;
    double scaleY = (double)WIN_H / FIELD_SIZE_MM;
    pixels_per_mm = std::min(scaleX, scaleY);

    // center offsets to place world centered
    double world_px_w = FIELD_SIZE_MM * pixels_per_mm;
    double offset_x_px = (WIN_W - world_px_w) / 2.0;
    double offset_y_px = (WIN_H - world_px_w) / 2.0; // keep square mapping

    auto worldToScreenX = [&](double wx)->int {
        return (int)std::round(offset_x_px + wx * pixels_per_mm);
    };
    auto worldToScreenY = [&](double wy)->int {
        // invert Y to have origin at bottom-left in world coords
        return (int)std::round(offset_y_px + (FIELD_SIZE_MM - wy) * pixels_per_mm);
    };

    // Initialize robot truth somewhere (choose non-centered)
    Robot truth{FIELD_SIZE_MM*0.25, FIELD_SIZE_MM*0.2, 0.9};

    // Initialize particles
    int cur_particles = INIT_PARTICLES;
    std::vector<Particle> parts;
    init_particles(parts, cur_particles, truth, 0.0, 0.0, FIELD_SIZE_MM, FIELD_SIZE_MM);

    // Noise dists
    std::normal_distribution<double> odom_trans_noise(0.0, ODOM_TRANS_STD);
    std::normal_distribution<double> odom_rot_noise(0.0, ODOM_ROT_STD);
    std::normal_distribution<double> sensor_noise(0.0, SENSOR_STD);

    bool quit = false;
    double est_x=truth.x, est_y=truth.y, est_th=truth.th;

    // keyboard state (simple per-frame read)
    while(!WindowShouldClose() && !quit){
        // handle input (WASD)
        double cmd_fwd = 0.0;
        double cmd_rot = 0.0;
        if(IsKeyDown(KEY_W)) cmd_fwd += CMD_FWD;
        if(IsKeyDown(KEY_S)) cmd_fwd -= CMD_FWD;
        if(IsKeyDown(KEY_A)) cmd_rot += CMD_ROT;
        if(IsKeyDown(KEY_D)) cmd_rot -= CMD_ROT;
        if(IsKeyPressed(KEY_Q)) { quit = true; break; }

        // Apply to truth
        double tdx = cmd_fwd * cos(truth.th);
        double tdy = cmd_fwd * sin(truth.th);
        truth.x += tdx;
        truth.y += tdy;
        truth.th = angnorm(truth.th + cmd_rot);
        // clamp
        truth.x = std::min(std::max(truth.x, 1.0), FIELD_SIZE_MM-1.0);
        truth.y = std::min(std::max(truth.y, 1.0), FIELD_SIZE_MM-1.0);

        // Simulated sensor measurements on truth
        double meas[3];
        for(int si=0; si<3; ++si) {
            double sx = truth.x + cos(truth.th)*SENSORS[si].dx - sin(truth.th)*SENSORS[si].dy;
            double sy = truth.y + sin(truth.th)*SENSORS[si].dx + cos(truth.th)*SENSORS[si].dy;
            double sang = angnorm(truth.th + SENSORS[si].angle);
            double md = raycast(walls, sx, sy, sang, MAX_SENSOR_RANGE);
            md += sensor_noise(rng); // noisy measurement
            if(md < 0.0) md = 0.0;
            if(md > MAX_SENSOR_RANGE) md = MAX_SENSOR_RANGE;
            meas[si] = md;
        }

        // Odometry delta (simulate encoder+IMU with noise)
        double odo_forward = sqrt(tdx*tdx + tdy*tdy) + odom_trans_noise(rng);
        double odo_rot = cmd_rot + odom_rot_noise(rng);

        // Motion update for particles
        std::normal_distribution<double> trans_noise(0.0, ODOM_TRANS_STD);
        std::normal_distribution<double> rot_noise(0.0, ODOM_ROT_STD);
        for(auto &p: parts) {
            double nf = odo_forward + trans_noise(rng);
            double nr = odo_rot + rot_noise(rng);
            p.x += nf * cos(p.th);
            p.y += nf * sin(p.th);
            p.th = angnorm(p.th + nr);
            // keep inside bounds
            if(p.x < 0) p.x = 0; if(p.x > FIELD_SIZE_MM) p.x = FIELD_SIZE_MM;
            if(p.y < 0) p.y = 0; if(p.y > FIELD_SIZE_MM) p.y = FIELD_SIZE_MM;
        }

        // Sensor update (weighting)
        double total_w = 0.0;
        for(auto &p: parts) {
            double w = 1.0;
            for(int si=0; si<3; ++si) {
                double psx = p.x + cos(p.th)*SENSORS[si].dx - sin(p.th)*SENSORS[si].dy;
                double psy = p.y + sin(p.th)*SENSORS[si].dx + cos(p.th)*SENSORS[si].dy;
                double psang = angnorm(p.th + SENSORS[si].angle);
                double expected = raycast(walls, psx, psy, psang, MAX_SENSOR_RANGE);
                double p_s = gaussian_pdf(meas[si] - expected, SENSOR_STD);
                w *= p_s;
            }
            p.w = w;
            total_w += p.w;
        }
        // Normalize weights (with floor)
        if(total_w <= 0.0) {
            // reinitialize weights uniformly
            double u = 1.0 / parts.size();
            for(auto &p: parts) p.w = u;
        } else {
            for(auto &p: parts) p.w /= total_w;
        }

        // Estimate pose
        estimate_pose(parts, est_x, est_y, est_th);

        // Adaptive particle count: compute Neff
        double neff = effective_sample_size(parts);
        double neff_ratio = neff / parts.size();

        // If Neff low, resample and (optionally) increase particle count
        if(neff_ratio < RESAMPLE_NEFF_RATIO) {
            // resample
            parts = systematic_resample(parts);
            // increase particle count moderately up to max
            int newN = std::min((int)std::round(parts.size() * 1.15) , MAX_PARTICLES);
            if(newN > (int)parts.size()) {
                // duplicate some particles with jitter
                std::uniform_int_distribution<int> pick(0, parts.size()-1);
                for(int i=parts.size(); i<newN; ++i) {
                    Particle np = parts[pick(rng)];
                    // jitter position slightly
                    std::normal_distribution<double> jitter(0.0, 8.0);
                    np.x = std::min(std::max(0.0, np.x + jitter(rng)), FIELD_SIZE_MM);
                    np.y = std::min(std::max(0.0, np.y + jitter(rng)), FIELD_SIZE_MM);
                    np.th = angnorm(np.th + std::normal_distribution<double>(0.0, 2.0*M_PI/180.0)(rng));
                    np.w = 1.0 / newN;
                    parts.push_back(np);
                }
                // renormalize weights
                double inv = 1.0 / parts.size();
                for(auto &p: parts) p.w = inv;
            }
        } else {
            // If Neff high, we can reduce particle count for speed
            int newN = std::max((int)std::round(parts.size() * 0.95), MIN_PARTICLES);
            if(newN < (int)parts.size()) {
                // simple downsample: keep highest-weight particles
                std::sort(parts.begin(), parts.end(), [](const Particle &a, const Particle &b){ return a.w > b.w; });
                parts.resize(newN);
                // renormalize
                double sumw = 0.0;
                for(auto &p: parts) sumw += p.w;
                for(auto &p: parts) p.w /= sumw;
            }
        }

        // Render
        BeginDrawing();
        ClearBackground(RAYWHITE);

        // draw background field scaled
        // draw texture to fill the square centered region
        DrawTexturePro(fieldTex,
                       (Rectangle){0.0f,0.0f,(float)fieldTex.width,(float)fieldTex.height},
                       (Rectangle){(float)offset_x_px,(float)offset_y_px,(float)(FIELD_SIZE_MM*pixels_per_mm),(float)(FIELD_SIZE_MM*pixels_per_mm)},
                       (Vector2){0,0}, 0.0f, WHITE);

        // draw walls (thin black lines)
        for(const auto &w: walls) {
            int x0 = worldToScreenX(w.x1), y0 = worldToScreenY(w.y1);
            int x1 = worldToScreenX(w.x2), y1 = worldToScreenY(w.y2);
            DrawLine(x0,y0,x1,y1, BLACK);
        }

        // draw particles (small circles, alpha by weight)
        for(const auto &p: parts) {
            int px = worldToScreenX(p.x), py = worldToScreenY(p.y);
            // map weight to color alpha (scale)
            float alpha = (float)std::min(1.0, p.w * parts.size() * 5.0);
            Color c = { (unsigned char)(30*(1-alpha)+10*alpha),
                        (unsigned char)(120*(1-alpha)+40*alpha),
                        (unsigned char)(220*(1-alpha)+200*alpha),
                        (unsigned char)(255*alpha) };
            DrawPixel(px, py, c);
        }

        // draw true robot (green circle + heading)
        int tx = worldToScreenX(truth.x), ty = worldToScreenY(truth.y);
        DrawCircle(tx, ty, 6, GREEN);
        DrawLine(tx, ty, (int)(tx + cos(truth.th)*14), (int)(ty - sin(truth.th)*14), DARKGREEN);

        // draw sensors rays from true robot (for visualization)
        for(int si=0; si<3; ++si) {
            double sx = truth.x + cos(truth.th)*SENSORS[si].dx - sin(truth.th)*SENSORS[si].dy;
            double sy = truth.y + sin(truth.th)*SENSORS[si].dx + cos(truth.th)*SENSORS[si].dy;
            double sang = angnorm(truth.th + SENSORS[si].angle);
            double dist = raycast(walls, sx, sy, sang, MAX_SENSOR_RANGE);
            int s0x = worldToScreenX(sx), s0y = worldToScreenY(sy);
            int s1x = worldToScreenX(sx + cos(sang)*dist);
            int s1y = worldToScreenY(sy + sin(sang)*dist);
            DrawLine(s0x, s0y, s1x, s1y, RED);
        }

        // draw estimated pose (red)
        int ex = worldToScreenX(est_x), ey = worldToScreenY(est_y);
        DrawCircle(ex, ey, 6, RED);
        DrawLine(ex, ey, (int)(ex + cos(est_th)*14), (int)(ey - sin(est_th)*14), MAROON);

        // HUD text
        DrawText(TextFormat("Particles: %d  Neff: %.1f  Est: (%.0f,%.0f,%.1fdeg)",
                            (int)parts.size(), effective_sample_size(parts), est_x, est_y, est_th*180.0/M_PI),
                 10, 10, 12, DARKGRAY);

        EndDrawing();

        // small sleep handled by raylib FPS
    }

    UnloadTexture(fieldTex);
    CloseWindow();
    return 0;
}
