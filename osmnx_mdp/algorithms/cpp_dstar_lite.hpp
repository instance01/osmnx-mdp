#include <unordered_map>

class cpp_DStar_Lite {
    public:
        cpp_DStar_Lite();
        ~cpp_DStar_Lite();
        std::unordered_map<long, float> rhs = {};
        std::unordered_map<long, float> g = {};
        std::unordered_map<long, std::pair<float, float>> U = {};
        int k = 0;
        long start;
        long goal;
};
