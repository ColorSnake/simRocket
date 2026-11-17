#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace component {
using Time = int64_t;
inline Time timeFromSeconds(double seconds) {
    if (!std::isfinite(seconds) || seconds < 0 || seconds > 9e9)
        throw std::invalid_argument("Invalid component time");
    const double ns = seconds * 1e9;
    if (std::abs(ns - std::round(ns)) > 1e-3)
        throw std::invalid_argument("Component time must be representable in nanoseconds");
    return static_cast<Time>(std::llround(ns));
}
constexpr Time unbounded = std::numeric_limits<Time>::max();
enum class Quality { valid, invalid, uninitialized };
enum class Status { missing, invalid, stale, valid };
enum class Policy { latest, all_samples, events };
struct Envelope {
    std::string source;
    uint64_t sequence;
    Time sample_time, available_time, valid_until;
    Quality quality;
    std::optional<Time> effective_time;
};
template<class T> struct Sample { Envelope envelope; T value; };
template<class T> struct Read {
    Status status = Status::missing;
    std::shared_ptr<const Sample<T>> sample;
};
// Each input has its own queue/cursor. Publication never runs a consumer.
// An output represents one named producer and one typed contract.
template<class T> class Output;
template<class T> class Input {
    friend class Output<T>;
    std::string name_;
    Policy policy_;
    Time max_age_;
    bool required_;
    size_t capacity_;
    std::deque<std::shared_ptr<const Sample<T>>> queue_;
    std::shared_ptr<const Sample<T>> latest_;
    Time last_read_ = -1;
    void checkTime(Time now) {
        if (now < last_read_ || now < 0) throw std::invalid_argument(name_ + ": read clock moved backwards");
        last_read_ = now;
    }
    Read<T> classify(std::shared_ptr<const Sample<T>> value, Time now) const {
        Status status = !value ? Status::missing :
            value->envelope.quality != Quality::valid ? Status::invalid :
            now >= value->envelope.valid_until || now - value->envelope.sample_time > max_age_ ? Status::stale : Status::valid;
        if (required_ && status != Status::valid)
            throw std::runtime_error(name_ + ": " + (status == Status::missing ? "missing" : status == Status::invalid ? "invalid" : "stale") + " at " + std::to_string(now) + " ns");
        return {status, std::move(value)};
    }
public:
    Input(std::string name, Policy policy, Time max_age, bool required, size_t capacity)
        : name_(std::move(name)), policy_(policy), max_age_(max_age), required_(required), capacity_(capacity) {
        if (max_age < 0 || !capacity) throw std::invalid_argument("Invalid input policy: " + name_);
    }
    Read<T> latest(Time now) {
        checkTime(now);
        if (policy_ != Policy::latest) throw std::logic_error("Input is not latest: " + name_);
        while (!queue_.empty() && queue_.front()->envelope.available_time <= now) {
            latest_ = queue_.front(); queue_.pop_front();
        }
        return classify(latest_, now);
    }
    std::vector<Read<T>> take(Time now) {
        checkTime(now);
        if (policy_ == Policy::latest) throw std::logic_error("Input is not queued: " + name_);
        std::vector<Read<T>> result;
        for (auto it = queue_.begin(); it != queue_.end();) {
            const auto& envelope = (*it)->envelope;
            if (envelope.available_time > now) break;
            if (envelope.effective_time && *envelope.effective_time > now) { ++it; continue; }
            auto value = *it;
            result.push_back(classify(value, now)); // Do not consume a failed required input.
            it = queue_.erase(it); latest_ = std::move(value);
        }
        if (result.empty() && policy_ != Policy::events) classify(latest_, now);
        return result;
    }
};
template<class T> class Output {
    std::string source_;
    uint64_t sequence_ = 0;
    std::function<void(const Sample<T>&)> publication_observer_;
    std::vector<std::weak_ptr<Input<T>>> inputs_;
public:
    explicit Output(std::string source) : source_(std::move(source)) {}
    // Passive diagnostic tap: no receiver queue, called once per publication.
    void observePublications(std::function<void(const Sample<T>&)> observer) {
        publication_observer_ = std::move(observer);
    }
    std::shared_ptr<Input<T>> connect(std::string name, Policy policy = Policy::latest,
            Time max_age = unbounded, bool required = true, size_t capacity = 4096) {
        auto input = std::make_shared<Input<T>>(std::move(name), policy, max_age, required, capacity);
        inputs_.push_back(input); return input;
    }
    void publish(const T& value, Time sample, Time available, Time valid_until = unbounded, Quality quality = Quality::valid, std::optional<Time> effective = std::nullopt) {
        if (sample < 0 || sample > available || available >= valid_until || (effective && *effective < available))
            throw std::invalid_argument(source_ + ": invalid envelope times");
        std::erase_if(inputs_, [](const auto& p) { return p.expired(); });
        // Preflight all destinations before committing any: no partial fan-out.
        for (auto& weak : inputs_) if (auto input = weak.lock())
            if (input->queue_.size() >= input->capacity_) throw std::runtime_error(input->name_ + ": queue overflow");
        auto packet = std::make_shared<const Sample<T>>(Sample<T>{{source_, sequence_++, sample, available, valid_until, quality, effective}, value});
        if (publication_observer_) publication_observer_(*packet);
        for (auto& weak : inputs_) if (auto input = weak.lock()) {
            auto position = std::upper_bound(input->queue_.begin(), input->queue_.end(), available,
                [](Time time, const auto& p) { return time < p->envelope.available_time; });
            input->queue_.insert(position, packet);
        }
    }
};
// Bus and direct connections share envelope rules, but are separate typed ports.
template<class T> using BusTopic = Output<T>;
template<class T> using DirectPort = Output<T>;

// Events are retained until their effective time and consumed once per subscriber.
// An event port has one receiver activation grid, checked before publication.
template<class T> struct Event { T action; Time available_time, effective_time; };
template<class T> class EventPort {
    Output<Event<T>> output_;
    Time receiver_period_;
public:
    EventPort(std::string source, Time period) : output_(std::move(source)), receiver_period_(period) {
        if (period <= 0) throw std::invalid_argument("Invalid event receiver period");
    }
    auto connect(std::string name, size_t capacity = 4096) {
        return output_.connect(std::move(name), Policy::events, unbounded, false, capacity);
    }
    void publish(const T& action, Time available, Time effective) {
        if (effective < available || effective % receiver_period_) throw std::invalid_argument("Event is off receiver grid");
        output_.publish({action, available, effective}, available, available, unbounded, Quality::valid, effective);
    }
};
}
