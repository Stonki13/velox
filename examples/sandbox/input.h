#pragma once

#include <array>

namespace sandbox {

class Window;

enum class Action : unsigned char {
    Forward,
    Backward,
    Left,
    Right,
    TurnLeft,
    TurnRight,
    Spawn,
    Reset,
    Pause,
    IncreaseSubsteps,
    DecreaseSubsteps,
    Stack,
    Rain,
    Ragdoll,
    Contraption,
    ToggleLines,
    Shape1,
    Shape2,
    Shape3,
    Shape4,
    Shape5,
    Shape6,
    Shape7,
    Count
};

class Input {
public:
    void update(const Window& window);
    bool down(Action action) const;
    bool pressed(Action action) const;
    bool mouseLook() const { return mouseLook_; }
    bool dragDown() const { return drag_; }
    bool dragPressed() const { return drag_ && !previousDrag_; }
    bool dragReleased() const { return !drag_ && previousDrag_; }
    float mouseDeltaX() const { return mouseDeltaX_; }
    float mouseDeltaY() const { return mouseDeltaY_; }
    float scrollDelta() const { return scrollDelta_; }
    double cursorX() const { return cursorX_; }
    double cursorY() const { return cursorY_; }

private:
    static constexpr size_t ActionCount = static_cast<size_t>(Action::Count);
    std::array<bool, ActionCount> down_{};
    std::array<bool, ActionCount> previous_{};
    bool mouseLook_ = false;
    bool previousMouseLook_ = false;
    bool drag_ = false;
    bool previousDrag_ = false;
    bool haveCursor_ = false;
    bool scrollHooked_ = false;
    double cursorX_ = 0.0;
    double cursorY_ = 0.0;
    double previousCursorX_ = 0.0;
    double previousCursorY_ = 0.0;
    float mouseDeltaX_ = 0.0f;
    float mouseDeltaY_ = 0.0f;
    float scrollDelta_ = 0.0f;
};

} // namespace sandbox
