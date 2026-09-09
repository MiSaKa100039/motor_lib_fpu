#include "App/Core/App_CommandInbox.h"

namespace UserAPP
{

namespace
{

AppCommandInbox g_command_inbox;

} // namespace

bool AppCommandInbox::push(const AppCommand& cmd)
{
    if (full())
    {
        return false;
    }

    queue_[tail_] = cmd;
    tail_ = static_cast<uint8_t>((tail_ + 1U) % APP_COMMAND_INBOX_CAPACITY);
    ++count_;
    return true;
}

bool AppCommandInbox::pop(AppCommand& out_cmd)
{
    if (empty())
    {
        return false;
    }

    out_cmd = queue_[head_];
    head_ = static_cast<uint8_t>((head_ + 1U) % APP_COMMAND_INBOX_CAPACITY);
    --count_;
    return true;
}

void AppCommandInbox::clear()
{
    head_ = 0U;
    tail_ = 0U;
    count_ = 0U;
}

bool AppCommandInbox::empty() const
{
    return count_ == 0U;
}

bool AppCommandInbox::full() const
{
    return count_ >= APP_COMMAND_INBOX_CAPACITY;
}

bool App_PostCommand(const AppCommand& cmd)
{
    return g_command_inbox.push(cmd);
}

bool App_TryTakeCommand(AppCommand& out_cmd)
{
    return g_command_inbox.pop(out_cmd);
}

void App_ClearCommands()
{
    g_command_inbox.clear();
}

} // namespace UserAPP
