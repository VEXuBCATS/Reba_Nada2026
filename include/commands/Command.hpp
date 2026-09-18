#ifndef COMMAND_HPP
#define COMMAND_HPP

class Command {
public:
    virtual ~Command() = default;
    virtual void initialize() {}       // Se ejecuta una vez al iniciar
    virtual void execute() {}          // Se ejecuta en bucle repetitivo
    virtual bool isFinished() = 0;     // Retorna true cuando el comando debe terminar
    virtual void end(bool interrupted) {} // Limpieza al finalizar
};

#endif