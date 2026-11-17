#pragma once
#include <algorithm>
#include <any>
#include <functional>
#include <memory>
#include <typeindex>
#include <unordered_map>
#include <vector>

// Synchronous, single-threaded dispatch. HIL threads must enqueue into the
// simulation thread. Scoped subscriptions disconnect when their owner dies.
class MessageBus {
    using Callback = std::function<void(const std::any&)>;
public:
    using Subscription = std::shared_ptr<Callback>;
    template <typename Message> Subscription subscribeScoped(std::function<void(const Message&)> callback) {
        auto slot = std::make_shared<Callback>([callback=std::move(callback)](const std::any& message) {
            callback(std::any_cast<const Message&>(message));
        });
        subscribers_[std::type_index(typeid(Message))].push_back(slot);
        return slot;
    }
    // Convenience for observers that explicitly live as long as the bus.
    template <typename Message> void subscribe(std::function<void(const Message&)> callback) {
        permanent_.push_back(subscribeScoped<Message>(std::move(callback)));
    }
    template <typename Message> void publish(const Message& message) {
        auto it = subscribers_.find(std::type_index(typeid(Message)));
        if (it == subscribers_.end()) return;
        auto& slots = it->second;
        std::erase_if(slots, [](const auto& slot) { return slot.expired(); });
        auto snapshot = slots; // Subscribing during a callback affects the next publication.
        const std::any value = message;
        for (const auto& weak : snapshot) if (auto slot = weak.lock()) (*slot)(value);
    }
private:
    std::unordered_map<std::type_index, std::vector<std::weak_ptr<Callback>>> subscribers_;
    std::vector<Subscription> permanent_;
};
