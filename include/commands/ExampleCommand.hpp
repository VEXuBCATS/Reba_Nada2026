#include "main.h"
#include "command.hpp"

extern pros::Motor motor;

class ExampleCommand : public Command {
private:
    int velocidad;
    uint32_t duracionMs;
    uint32_t tiempoInicio;

public:
    ExampleCommand(int velocidad, uint32_t duracionMs) 
        : velocidad(velocidad), duracionMs(duracionMs) {}

    void initialize() override {
        tiempoInicio = pros::millis();
    }

    void execute() override {
        motor.move(velocidad);
    }

    bool isFinished() override {
        return (pros::millis() - tiempoInicio) >= duracionMs;
    }

    void end(bool interrupted) override {
        motor.move(0);
    }
};