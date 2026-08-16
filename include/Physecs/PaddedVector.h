#pragma once

#include <algorithm>

// array for which it is safe to index at -1
template<typename T>
class PaddedVector {
    T* data;
    size_t size;
public:
    PaddedVector() {
        T* p = new T[1]{};
        data = p + 1;
        size = 0;
    }
    T& operator[](int i) { return data[i]; }
    void resize(int newSize) {
        if (newSize > size) {
            T* newData = new T[newSize + 1];
            std::copy_n(data - 1, size + 1, newData);
            delete[](data - 1);
            data = newData + 1;
            size = newSize;
        }
    }
    T* getData() { return data; }
    ~PaddedVector() {
        delete[](data - 1);
    }
};
