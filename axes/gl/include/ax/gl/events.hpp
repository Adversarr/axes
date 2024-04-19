#pragma once

namespace ax::gl {

struct TickLogicEvent{};
struct UiRenderEvent{};
struct ContextDestroyEvent {};
struct ContextInitEvent {};
struct MainMenuBarRenderEvent {};

constexpr TickLogicEvent tick_logic_event{};
constexpr UiRenderEvent ui_render_event{};
constexpr ContextDestroyEvent context_destroy_event{};
constexpr ContextInitEvent context_init_event{};
constexpr MainMenuBarRenderEvent main_menu_bar_render_event{};

}