#ifndef STEREO_ROTATING_PLATE_CALIBRATION__THREAD_SAFE_QUEUE_HPP_
#define STEREO_ROTATING_PLATE_CALIBRATION__THREAD_SAFE_QUEUE_HPP_

#include <queue>
#include <mutex>
#include <condition_variable>
#include <chrono>

template <typename T>
class ThreadSafeQueue {
public:
    void push(const T& value) {
        std::unique_lock<std::mutex> lock(mutex_);
        queue_.push_back(value);
        lock.unlock();
        cond_var_.notify_one();
    }


    T pop() {
        std::unique_lock<std::mutex> lock(mutex_);
        cond_var_.wait(lock, [this] { return !queue_.empty(); });
        T value = queue_.front();
        queue_.pop_front();
        return value;
    }

    bool try_pop_for(T& value, long long timeout_ms) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (cond_var_.wait_for(lock, std::chrono::milliseconds(timeout_ms), [this] { return !queue_.empty(); })) {
            value = queue_.front();
            queue_.pop_front();
            return true;
        }
        return false;
    }

    bool empty() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }

    void clear() {
        std::unique_lock<std::mutex> lock(mutex_);
        queue_.clear();
    }

    // size_t size() const {
    //     std::lock_guard<std::mutex> lock(mutex_);
    //     return queue_.size();
    // }
    int size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return static_cast<int>(queue_.size());
    }

    T back() {
        std::unique_lock<std::mutex> lock(mutex_);
        cond_var_.wait(lock, [this] { return !queue_.empty(); });
        return queue_.back();
    }

    T front() {
        std::unique_lock<std::mutex> lock(mutex_);
        cond_var_.wait(lock, [this] { return !queue_.empty(); });
        return queue_.front();
    }

    T operator[](size_t index) const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (index >= queue_.size()) {
            throw std::out_of_range("ThreadSafeDeque::operator[]: index out of bounds");
        }
        return queue_[index];
    }

    std::deque<T> snapshot() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_;
    }

private:
    std::deque<T> queue_;
    mutable std::mutex mutex_;
    std::condition_variable cond_var_;
};


#endif