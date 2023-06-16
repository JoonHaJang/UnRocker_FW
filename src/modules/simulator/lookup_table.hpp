
#include <vector>
#include <unordered_map>

class LookupTable {
public:
    void insert(int index, float value);
    float get(int index);

private:
    std::unordered_map<int, std::vector<float>> table;
};

void LookupTable::insert(int index, float value) {
    if (table.find(index) == table.end()) {
        table[index] = std::vector<float>();
    }
    table[index].push_back(value);
}

float LookupTable::get(int index) {
    if (table.find(index) == table.end() || table[index].empty()) {
        return 0.0f;
    }
    return table[index].back();
}

