#include "commands/CommandScheduler.hpp"

CommandScheduler scheduler;

void CommandScheduler::schedule(std::shared_ptr<Command> cmd) {
    if (cmd) {
        cmd->initialize();
        comandosActivos.push_back(cmd);
    }
}

void CommandScheduler::run() {
    for (auto it = comandosActivos.begin(); it != comandosActivos.end(); ) {
        auto cmd = *it;
        cmd->execute();

        if (cmd->isFinished()) {
            cmd->end(false);
            it = comandosActivos.erase(it);
        } else {
            ++it;
        }
    }
}