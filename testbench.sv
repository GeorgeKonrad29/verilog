module monociclo_tb;
    // Señales de prueba
    logic clk;
    logic reset;
    logic [31:0] pc;
    logic [31:0] instruction;
    logic [31:0] result;
    logic rs1R;
    logic rs2R;
    logic [31:0] rs1_data;
    logic [31:0] rs2_data;
    logic [31:0] datos_alRegistro;

    // Instancia del módulo monociclo
    monociclo uut (
        .clk(clk),
        .reset(reset),
        .pc(pc),
        .instruction(instruction),
        .result(result),
        .rs1R(rs1R),
        .rs2R(rs2R),
        .rs1_data(rs1_data),
        .rs2_data(rs2_data),
        .datos_alRegistro(datos_alRegistro)
    );

    // Generar el reloj
    always #5 clk = ~clk;

    // Inicializar las señales de prueba
    initial begin
        // Inicializar el reloj y reset
        clk = 0;
        reset = 1;

        // Esperar unos ciclos de reloj
        #10;
        reset = 0;

        // Esperar más ciclos de reloj para observar el comportamiento
        #100;

        // Finalizar la simulación
        $finish;
    end

    // Monitor para observar las señales
    initial begin
        $monitor("Time: %0t | clk: %b | reset: %b | pc: %h | instruction: %h | result: %h | rs1R: %b | rs2R: %b | rs1_data: %h | rs2_data: %h | datos_alRegistro: %h", 
                 $time, clk, reset, pc, instruction, result, rs1R, rs2R, rs1_data, rs2_data, datos_alRegistro);
    end
endmodule