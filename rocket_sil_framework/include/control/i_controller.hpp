#pragma once

class IController {
public:
    virtual ~IController() = default;
    
    // Called once per logical tick
    virtual void update(double dt) = 0;
};
