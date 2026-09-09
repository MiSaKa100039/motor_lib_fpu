#pragma once

#include "App/Core/App_Command.h"
#include "App/Core/App_Config.h"

namespace UserAPP
{

class AppCommandInbox
{
public:
    bool push(const AppCommand& cmd);
    bool pop(AppCommand& out_cmd);
    void clear();
    bool empty() const;
    bool full() const;

private:
    AppCommand queue_[APP_COMMAND_INBOX_CAPACITY];
    uint8_t head_ = 0U;
    uint8_t tail_ = 0U;
    uint8_t count_ = 0U;
};

bool App_PostCommand(const AppCommand& cmd);
bool App_TryTakeCommand(AppCommand& out_cmd);
void App_ClearCommands();

} // namespace UserAPP
