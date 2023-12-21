function [system, system_nom] = system_gen(plant, input, output)

    system = ss(plant.A, plant.B, plant.C, plant.D);
    system.u = input;
    system.y = output;

    if isprop(system, 'NominalValue')
        system_nom = system.NominalValue;
        system_nom.u = input;
        system_nom.y = output;
    else
        system_nom = system;
        system_nom.u = input;
        system_nom.y = output;
    end
