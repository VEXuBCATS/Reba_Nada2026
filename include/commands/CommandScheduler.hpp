#ifndef COMMAND_SCHEDULER_HPP
#define COMMAND_SCHEDULER_HPP

#include <vector>
#include <memory>
#include "command.hpp"

class CommandScheduler {
private:
    std::vector<std::shared_ptr<Command>> comandosActivos;

public:
    void schedule(std::shared_ptr<Command> cmd);
    void run();
};

extern CommandScheduler scheduler;

#endif