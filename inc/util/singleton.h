#ifndef _INTERFACE_SINGLETON_H_
#define _INTERFACE_SINGLETON_H_

#include <memory>
#include <mutex>

template<typename T>
class Singleton {
public:
    // Deleted copy constructor and assignment operator to prevent copying
    Singleton(const Singleton&) = delete;
    auto operator=(const Singleton&) -> Singleton& = delete;

    // Accessor method to get the singleton instance
    static auto getInstance() -> T* {
        std::call_once(initFlag, []() {
            instance.reset(new T());
        });
        return instance.get();
    }

protected:
    Singleton() = default; // Private constructor

private:
    static std::unique_ptr<T> instance; // Unique pointer to hold the singleton instance
    static std::once_flag initFlag; // Flag to ensure initialization is done only once
};

// Initialize static members
template<typename T>
std::unique_ptr<T> Singleton<T>::instance;

template<typename T>
std::once_flag Singleton<T>::initFlag;
#endif