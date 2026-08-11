#pragma once

#include <functional>
#include <vector>
#include <typeindex>
#include <unordered_map>
#include <any>
#include <memory>

class MessageBus {
public:
    // Subscribe to a specific message type
    template <typename MessageType>
    void subscribe(std::function<void(const MessageType&)> callback) {
        auto type_idx = std::type_index(typeid(MessageType));
        
        // Wrap the strongly-typed callback into a type-erased callback using std::any
        auto wrapped_callback = [callback](const std::any& msg_any) {
            callback(std::any_cast<MessageType>(msg_any));
        };
        
        subscribers_[type_idx].push_back(wrapped_callback);
    }

    // Publish a specific message type
    template <typename MessageType>
    void publish(const MessageType& message) {
        auto type_idx = std::type_index(typeid(MessageType));
        
        if (subscribers_.find(type_idx) != subscribers_.end()) {
            std::any msg_any = message;
            for (auto& callback : subscribers_[type_idx]) {
                callback(msg_any);
            }
        }
    }

private:
    std::unordered_map<std::type_index, std::vector<std::function<void(const std::any&)>>> subscribers_;
};
