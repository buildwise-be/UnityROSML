// example_task.cpp
//
// Template for the actual work executable launched by remote_launcher.
// It receives the JSON payload from Unity on stdin (one line), extracts the
// poses, and does whatever it needs to do with them.
//
// The parsing below is a minimal hand-rolled extractor good enough for the
// fixed payload produced by Unity's JsonUtility. For anything more elaborate,
// drop in a real JSON library such as nlohmann/json
// (https://github.com/nlohmann/json, single header) and replace parsePoses().
//
// Build:
//   Linux:   g++ -std=c++17 -O2 -o example_task example_task.cpp
//   Windows: cl /std:c++17 /O2 example_task.cpp

#include <iostream>
#include <string>
#include <vector>

struct PoseStamped
{
    std::string frame_id;
    unsigned long sec = 0;
    unsigned long nanosec = 0;
    double px = 0, py = 0, pz = 0;      // position
    double qx = 0, qy = 0, qz = 0, qw = 1; // orientation
};

// Finds "key" after position `from` and parses the number that follows it.
static double numberAfter(const std::string& s, const std::string& key, size_t from, size_t end)
{
    size_t k = s.find("\"" + key + "\":", from);
    if (k == std::string::npos || k > end)
        return 0.0;
    return std::atof(s.c_str() + k + key.size() + 3);
}

static std::string stringAfter(const std::string& s, const std::string& key, size_t from, size_t end)
{
    size_t k = s.find("\"" + key + "\":\"", from);
    if (k == std::string::npos || k > end)
        return "";
    size_t start = k + key.size() + 4;
    size_t stop = s.find('"', start);
    return s.substr(start, stop - start);
}

static std::vector<PoseStamped> parsePoses(const std::string& json)
{
    std::vector<PoseStamped> poses;

    // Each pose object starts with "frame_id".
    size_t cursor = 0;
    while (true)
    {
        size_t start = json.find("\"frame_id\"", cursor);
        if (start == std::string::npos)
            break;
        size_t next = json.find("\"frame_id\"", start + 1);
        size_t end = (next == std::string::npos) ? json.size() : next;

        PoseStamped p;
        p.frame_id = stringAfter(json, "frame_id", start, end);
        p.sec = (unsigned long)numberAfter(json, "sec", start, end);
        p.nanosec = (unsigned long)numberAfter(json, "nanosec", start, end);

        size_t posBlock = json.find("\"position\"", start);
        size_t oriBlock = json.find("\"orientation\"", start);

        p.px = numberAfter(json, "x", posBlock, oriBlock);
        p.py = numberAfter(json, "y", posBlock, oriBlock);
        p.pz = numberAfter(json, "z", posBlock, oriBlock);

        p.qx = numberAfter(json, "x", oriBlock, end);
        p.qy = numberAfter(json, "y", oriBlock, end);
        p.qz = numberAfter(json, "z", oriBlock, end);
        p.qw = numberAfter(json, "w", oriBlock, end);

        poses.push_back(p);
        cursor = end;
    }
    return poses;
}

int main()
{
    // remote_launcher pipes the JSON payload as a single line on stdin.
    std::string json;
    if (!std::getline(std::cin, json) || json.empty())
    {
        std::cerr << "example_task: no input received on stdin\n";
        return 2;
    }

    std::vector<PoseStamped> poses = parsePoses(json);
    std::cout << "example_task: received " << poses.size() << " pose(s)\n";

    for (size_t i = 0; i < poses.size(); ++i)
    {
        const PoseStamped& p = poses[i];
        std::cout << "  Pose " << i
                  << " [" << p.frame_id << " @ " << p.sec << "." << p.nanosec << "]"
                  << "  pos(" << p.px << ", " << p.py << ", " << p.pz << ")"
                  << "  quat(" << p.qx << ", " << p.qy << ", " << p.qz << ", " << p.qw << ")\n";
    }

    // ======================================================================
    // TODO: put your actual task here (motion computation, logging, whatever
    // this executable is supposed to do with the poses).
    // ======================================================================

    return 0; // 0 => Unity gets {"ok":true,"exit_code":0}
}
