#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>
#include <thread>
#include <algorithm>

const double PI = 3.14159265359;

struct Vec3 {
    double x, y, z;
    Vec3(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}
    Vec3 operator-(const Vec3& v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
    Vec3 operator+(const Vec3& v) const { return Vec3(x + v.x, y + v.y, z + v.z); }
    Vec3 operator*(double s) const { return Vec3(x * s, y * s, z * s); }
    double dot(const Vec3& v) const { return x * v.x + y * v.y + z * v.z; }
    Vec3 cross(const Vec3& v) const { return Vec3(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x); }
    Vec3 normalize() const {
        double mag = sqrt(x*x + y*y + z*z);
        return Vec3(x/mag, y/mag, z/mag);
    }
};

void clearScreen() {
    std::cout << "\033[2J\033[H";
}

void drawPixel(std::vector<std::vector<char>>& screen, int x, int y, char pixel) {
    if (y >= 0 && y < screen.size() && x >= 0 && x < screen[0].size()) {
        screen[y][x] = pixel;
    }
}

void drawLine(std::vector<std::vector<char>>& screen, int x0, int y0, int x1, int y1, char pixel) {
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2;

    while (true) {
        drawPixel(screen, x0, y0, pixel);
        if (x0 == x1 && y0 == y1) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

void rotatePoint(Vec3& v, double angleX, double angleY, double angleZ) {
    // Rotation around X-axis
    double y = v.y;
    double z = v.z;
    v.y = y * cos(angleX) - z * sin(angleX);
    v.z = y * sin(angleX) + z * cos(angleX);

    // Rotation around Y-axis
    double x = v.x;
    z = v.z;
    v.x = x * cos(angleY) - z * sin(angleY);
    v.z = x * sin(angleY) + z * cos(angleY);

    // Rotation around Z-axis
    x = v.x;
    y = v.y;
    v.x = x * cos(angleZ) - y * sin(angleZ);
    v.y = x * sin(angleZ) + y * cos(angleZ);
}

bool isFaceVisible(const Vec3& v1, const Vec3& v2, const Vec3& v3, const Vec3& cameraPos) {
    Vec3 normal = (v2 - v1).cross(v3 - v1).normalize();
    Vec3 viewVector = (cameraPos - ((v1 + v2 + v3) * (1.0/3.0))).normalize();
    return normal.dot(viewVector) < 0.1; // Increased threshold for better visibility
}

Vec3 projectPoint(const Vec3& point, const Vec3& cameraPos, double focalLength, int centerX, int centerY, double scale) {
    Vec3 p = point - cameraPos;
    double z = p.z + focalLength;
    double x = (p.x * focalLength / z) * scale + centerX;
    double y = (p.y * focalLength / z) * scale + centerY;
    return Vec3(x, y, z);
}

void drawCube(std::vector<std::vector<char>>& screen, const std::vector<Vec3>& vertices, const Vec3& cameraPos) {
    std::vector<std::vector<int>> faces = {
        {0,1,2,3}, {7,6,5,4}, {4,5,1,0}, {6,7,3,2}, {3,7,4,0}, {1,5,6,2}
    };

    int centerX = screen[0].size() / 2;
    int centerY = screen.size() / 2;
    double scale = std::min(centerX, centerY) / 2.0;
    double focalLength = 10.0; // Adjust this for different perspective effects

    std::vector<std::pair<double, std::vector<int>>> sortedFaces;
    for (const auto& face : faces) {
        Vec3 center = (vertices[face[0]] + vertices[face[1]] + vertices[face[2]] + vertices[face[3]]) * 0.25;
        double depth = (center - cameraPos).dot(center - cameraPos);
        sortedFaces.push_back({depth, face});
    }
    std::sort(sortedFaces.begin(), sortedFaces.end(), std::greater<>());

    for (const auto& [depth, face] : sortedFaces) {
        if (isFaceVisible(vertices[face[0]], vertices[face[1]], vertices[face[2]], cameraPos)) {
            for (int i = 0; i < 4; i++) {
                int j = (i + 1) % 4;
                Vec3 p1 = projectPoint(vertices[face[i]], cameraPos, focalLength, centerX, centerY, scale);
                Vec3 p2 = projectPoint(vertices[face[j]], cameraPos, focalLength, centerX, centerY, scale);
                drawLine(screen, p1.x, p1.y, p2.x, p2.y, 'X');
            }
        }
    }
}

int main() {
    const int width = 180;
    const int height = 180;
    std::vector<std::vector<char>> screen(height, std::vector<char>(width, ' ')); 

    std::vector<Vec3> vertices = {
        {-1,-1,-1}, {1,-1,-1}, {1,1,-1}, {-1,1,-1},
        {-1,-1,1}, {1,-1,1}, {1,1,1}, {-1,1,1}
    };

    Vec3 cameraPos(0, 0, -5);
    double angleX = 0.05, angleY = 0.10, angleZ = 0.02;
    
    while (true) {
        clearScreen();
        for (auto& row : screen) std::fill(row.begin(), row.end(), ' ');
        
        for (auto& v : vertices) {
            rotatePoint(v, angleX, angleY, angleZ);
        }
        drawCube(screen, vertices, cameraPos);
        
        for (const auto& row : screen) {
            for (char c : row) std::cout << c << ' ';  // Add space after each character
      //      std::cout << '\n';
        }
        
        std::cout.flush();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        angleX += 0.0;
        angleY += 0.0;
        angleZ += 0.0;
    }

    return 0;
}