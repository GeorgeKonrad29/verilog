// Code your design here
module branch_unit(
    input logic [31:0] rs1,
    input logic [31:0] rs2,
    input logic [4:0] br_op,
    output logic branch
);
    always_comb begin
        case (br_op)
            5'b00000: branch = 0; // No branch
            5'b01000: branch = (rs1 == rs2); // BEQ
            5'b01001: branch = (rs1 != rs2); // BNE
            5'b01100: branch = (rs1 < rs2); // BLT
            5'b01101: branch = (rs1 >= rs2); // BGE
            //las siguientes son sin signo
            5'b01110: branch = ($unsigned(rs1) < $unsigned(rs2)); // BLTU
            5'b01111: branch = ($unsigned(rs1) >= $unsigned(rs2)); // BGEU
            5'b11111: branch = 1; // JAL
            default: branch = 0; // No branch
        endcase
    end
endmodule
module data_memory (
    input logic clk,
    input logic mem_write,
    input logic [2:0] mem_ctrl, // Control de memoria
    input logic [31:0] address,
    input logic [31:0] write_data,
    output logic [31:0] read_data
);
    logic [7:0] memory [0:1023]; // Tamaño de memoria de ejemplo (1024 bytes)

    always_ff @(posedge clk) begin
        if (mem_write) begin
            case (mem_ctrl)
                3'b000: memory[address] <= write_data[7:0]; // Escribir byte
                3'b001: begin // Escribir media palabra
                    memory[address] <= write_data[7:0];
                    memory[address + 1] <= write_data[15:8];
                end
                3'b010: begin // Escribir palabra
                    memory[address] <= write_data[7:0];
                    memory[address + 1] <= write_data[15:8];
                    memory[address + 2] <= write_data[23:16];
                    memory[address + 3] <= write_data[31:24];
                end
                default: ; // No hacer nada para otros valores
            endcase
        end
    end

    always_comb begin
        case (mem_ctrl)
            3'b000: read_data = {{24{memory[address][7]}}, memory[address]}; // Leer byte
            3'b001: read_data = {{16{memory[address + 1][7]}}, memory[address + 1], memory[address]}; // Leer media palabra
            3'b010: read_data = {memory[address + 3], memory[address + 2], memory[address + 1], memory[address]}; // Leer palabra
            3'b100: read_data = {24'b0, memory[address]}; // Leer byte sin signo
            3'b101: read_data = {16'b0, memory[address + 1], memory[address]}; // Leer media palabra sin signo
            default: read_data = 32'b0; // No hacer nada para otros valores
        endcase
    end
endmodule
module control_unit (
    input logic [6:0] opCode,
    input logic [2:0] fun3,
    input logic [6:0] fun7,
    output logic DMWr,
    output logic [2:0] ImmSrc,
    output logic ALUBSrc,
    output logic [4:0] BrOp,
    output logic [1:0] RUDataWrSrc,
    output logic ALUASrc,
    output logic [3:0] ALUOpcode,
    output logic [2:0] DMCtrl,
    output logic RUWr
);
    always_comb begin
        // Inicializar todas las señales de control a 0
        DMWr = 0;
        ImmSrc = 3'b000;
        ALUBSrc = 0;
        BrOp = 5'b00000;
        RUDataWrSrc = 2'b00;
        ALUASrc = 0;
        ALUOpcode = 4'b0000;
        DMCtrl = 3'b000;
        RUWr = 0;

        case (opCode)
            7'b0110011: begin // R-type
                DMWr = 0;
       			ImmSrc = 3'b111;
        		ALUBSrc = 0;
        		BrOp = 5'b00000;
        		RUDataWrSrc = 2'b00;
        		ALUASrc = 0;
              	ALUOpcode = {fun7[5],fun3};
        		DMCtrl = 3'b111;
        		RUWr = 1;
                // Asignar valores adicionales según sea necesario
            end
            7'b0010011: begin // I-type
                DMWr = 0;
       			ImmSrc = 3'b000;
        		ALUBSrc = 1;
        		BrOp = 5'b00000;
        		RUDataWrSrc = 2'b00;
        		ALUASrc = 0;
                if(fun3 == 3'b101) begin
                	ALUOpcode = {fun7[5],fun3};
                end
                else begin
                	ALUOpcode = fun3;
                end
              	ALUOpcode = fun3;
        		DMCtrl = 3'b111;
        		RUWr = 1;
            end
            7'b0000011: begin // Load
                DMWr = 0;//no se escribe en memoria
                ImmSrc = 3'b000;//tipo i
                ALUBSrc = 1;//inmediato o rs2
                BrOp = 5'b00000;//pc normal
                RUDataWrSrc = 2'b01;//se escribe lo que haya en la memoria
                ALUASrc = 0;//rs1 o pc
                ALUOpcode = 4'b0000;//siempre es suma
                DMCtrl = fun3;//se carga el tipo de dato
                RUWr = 1;//se escribe en el registro
            end
            7'b0100011: begin // Store
                DMWr = 1;//se escribe en memoria
                ImmSrc = 3'b001;//tipo s
                ALUBSrc = 1;//inmediato o rs2
                BrOp = 5'b00000;
                RUDataWrSrc = 2'b00;
                ALUASrc = 0;
                ALUOpcode = 4'b0000;
                DMCtrl = fun3;
                RUWr = 0;
            end

            7'b1100011: begin // Branch
                DMWr = 0;
                ImmSrc = 3'b101;
                ALUBSrc = 1;
                BrOp = {0'b01,fun3[2:0]};
                RUDataWrSrc = 2'b00;
                ALUASrc = 1;
                ALUOpcode = 4'b0000;
                DMCtrl = 3'b111;
                RUWr = 0;
            end

            7'b1101111: begin // Jal
                DMWr = 0;
                ImmSrc = 3'b110;
                ALUBSrc = 1;
                BrOp = 5'b11111;
                RUDataWrSrc = 2'b10;
                ALUASrc = 1;
                ALUOpcode = 4'b0000;
                DMCtrl = 3'b000;
                RUWr = 1;
            end

            7'b1100111: begin // Jalr
                DMWr = 0;
                ImmSrc = 3'b000;
                ALUBSrc = 1;
                BrOp = 5'b11111;
                RUDataWrSrc = 2'b10;
                ALUASrc = 0;
                ALUOpcode = 4'b0000;
                DMCtrl = 3'b000;
                RUWr = 1;
            end


            // Agregar más casos para otros tipos de instrucciones según sea necesario
            default: begin
                // Mantener todas las señales de control en 0
            end
        endcase
    end
endmodule
module alu (
    input logic [31:0] operand1,
    input logic [31:0] operand2,
    input logic [3:0] alu_control,
    output logic [31:0] result,
    output logic zero
);
    always_comb begin
        case (alu_control)
            4'b0000: result = operand1 + operand2; // Suma
            4'b1000: result = operand1 - operand2; // Resta
            4'b0001: result = operand1 << operand2[4:0]; // Corrimiento a la izquierda
            4'b0010: result = (operand1 < operand2) ? 32'b1 : 32'b0; // Set less than
            4'b0011: result = ($unsigned(operand1) < $unsigned(operand2)) ? 32'b1 : 32'b0; // Set less than (unsigned)
            4'b0100: result = operand1 ^ operand2; // XOR
            4'b0101: result = operand1 >> operand2[4:0]; // Corrimiento a la derecha lógico
            4'b1101: result = $signed(operand1) >>> operand2[4:0]; // Corrimiento a la derecha aritmético
            4'b0110: result = operand1 | operand2; // OR
            4'b0111: result = operand1 & operand2; // AND
            default: result = 32'b0; // Operación no definida
        endcase
        zero = (result == 32'b0);
    end
endmodule
module immediate_generator (
    input logic [24:0] instruction,
    input logic [2:0] instr_type, // Nueva entrada para seleccionar el tipo de instrucción
    output logic [31:0] immediate
);
    always_comb begin
        // Inicializar el inmediato a 0
        
        // Generar el inmediato basado en el tipo de instrucción
        case (instr_type)
            3'b000: // Tipo I
              immediate = {{19{instruction[24]}}, instruction[24:13]};
            3'b001: // Tipo S
              immediate = {{20{instruction[24]}}, instruction[24:18], instruction[4:0]};
            3'b101: // Tipo B
                immediate = {{20{instruction[24]}} ,instruction[24], instruction[0], instruction[23:18], instruction[4:1], 1'b0};
            3'b110: // Tipo j
                immediate = {{12{instruction[24]}}, instruction[24],instruction[12:5], instruction[13], instruction[23:14], 1'b0};
            // Agregar más casos para otros tipos de instrucciones según sea necesario
            default:
                immediate = 32'b0;
        endcase
    end
endmodule
module register_unit (
    input logic clk,
    input logic [4:0] read_reg1,
    input logic [4:0] read_reg2,
    input logic [4:0] write_reg,
    input logic [31:0] write_data,
    input logic reg_write,
    output logic [31:0] read_data1,
    output logic [31:0] read_data2
);
    logic [31:0] registers [0:31]; // 32 registros de 32 bits

    // Leer datos de los registros
    assign read_data1 = registers[read_reg1];
    assign read_data2 = registers[read_reg2];
    // Escribir datos en los registros
    always_ff @(posedge clk) begin
      	
	  registers[0] <= 32'b0;
      if (reg_write)
            registers[write_reg] <= write_data;
        	
    end

    // Inicializar registros (opcional)
    initial begin
    end
endmodule
module program_counter (
    input logic clk,
    input logic reset,
    input logic [31:0] next_pc,
    output logic [31:0] pc
);
    always_ff @(posedge clk or posedge reset) begin
        if (reset)
            pc <= 32'b0;
        else
            pc <= next_pc;
    end
endmodule
module instruction_memory (
    input logic [31:0] address,
    output logic [31:0] instruction
);
    logic [7:0] memory [0:1023]; // Tamaño de memoria de ejemplo (1024 bytes)

    assign instruction = {memory[address], memory[address+1], memory[address+2], memory[address+3]}; // Mapeo simple de dirección

    initial begin
        // Cargar instrucciones en la memoria (ejemplo)
        memory[0] = 8'h13; // NOP (addi x0, x0, 0)
        memory[1] = 8'h00;
        memory[2] = 8'h00;
        memory[3] = 8'h00;
        
      	memory[4] = 8'h00; // addi x1, x0, 1
      	memory[5] = 8'h10;
      	memory[6] = 8'h00;
      	memory[7] = 8'h93;
        
      	memory[8] = 8'h00; // addi x2, x0, 2
      	memory[9] = 8'h20;
      	memory[10] = 8'h01;
      	memory[11] = 8'h13;

        memory[12] = 8'h00; // add x3, x2, x1
        memory[13] = 8'h20;
        memory[14] = 8'h81;
        memory[15] = 8'hb3;

        memory[16] = 8'h00; // sh x3, 0(x0)
        memory[17] = 8'h30;
        memory[18] = 8'h10;
        memory[19] = 8'h23;

        memory[20] = 8'h00; // lh x4, 0(x0)
        memory[21] = 8'h00;
        memory[22] = 8'h12;
        memory[23] = 8'h03;

      	memory[24] = 8'h00; // blt x3, x2, 8
      memory[25] = 8'h31;
      memory[26] = 8'h44;
        memory[27] = 8'h63;

        memory[28] = 8'h01; // jalr x6, 16(x0)
        memory[29] = 8'h00;
        memory[30] = 8'h03;
        memory[31] = 8'h67;

      	memory[32] = 8'hff; // jal x5, -4
      	memory[33] = 8'hdf;
        memory[34] = 8'hf2;
        memory[35] = 8'hef;
        // Agregar más instrucciones según sea necesario
    end
endmodule
module monociclo (
    input logic clk,
    input logic reset,
    output logic [31:0] pc,
    output logic [31:0] instruction,
    output logic [31:0] result,
    output logic rs1R,rs2R,
    output logic [31:0] rs1_data,rs2_data,
    output logic [31:0] datos_alRegistro 
);
    // Señales internas
    logic [31:0] next_pc;
    logic [31:0] pc_plus_4;
    logic [31:0] rs1;
    logic [31:0] rs2;
    logic [31:0] imm;
    logic [31:0] ALUASrc_result;
    logic [31:0] ALUBSrc_result;
    logic [6:0] opCode;
    logic DMWr;
    logic [2:0] ImmSrc;
    logic ALUBSrc;
    logic [4:0] BrOp;
    logic [1:0] RUDataWrSrc;
    logic ALUASrc;
    logic [3:0] ALUOpcode;
    logic [2:0] DMCtrl;
    logic RUWr;
    logic [31:0] dm_read_data;
    logic [31:0] ru_wr_src;
    logic [31:0] resultado_alu;
    logic [2:0] fun3;
    logic [6:0] fun7;
    logic branch;

    // Instancia del program counter
    program_counter pc_inst (
        .clk(clk),
        .reset(reset),
        .next_pc(next_pc),
        .pc(pc)
    );

    

    // Conectar pc a la entrada de instruction_memory
    instruction_memory im_inst (
        .address(pc),
        .instruction(instruction)
    );

    // Extraer el opCode de la instrucción
    assign opCode = instruction[6:0];
    assign fun3 = instruction[14:12];
    assign fun7 = instruction[31:25];
    // Instancia del control unit
    control_unit cu_inst (
        .opCode(opCode),
        .fun3(fun3),
        .fun7(fun7),
        .DMWr(DMWr),
        .ImmSrc(ImmSrc),
        .ALUBSrc(ALUBSrc),
        .BrOp(BrOp),
        .RUDataWrSrc(RUDataWrSrc),
        .ALUASrc(ALUASrc),
        .ALUOpcode(ALUOpcode),
        .DMCtrl(DMCtrl),
        .RUWr(RUWr)
    );
    

    register_unit ru_inst (
        .clk(clk),
        .read_reg1(instruction[19:15]),
        .read_reg2(instruction[24:20]),
        .write_reg(instruction[11:7]),
        .write_data(ru_wr_src),
        .reg_write(RUWr),
        .read_data1(rs1),
        .read_data2(rs2)
    );

    immediate_generator imm_gen_inst (
        .instruction(instruction[31:7]),
        .instr_type(ImmSrc),
        .immediate(imm)
    );

    branch_unit bu_inst (
        .rs1(rs1),
        .rs2(rs2),
        .br_op(BrOp),
        .branch(branch)
    );
    
    // Multiplexor ALUAsrc
  assign ALUASrc_result = (ALUASrc) ? pc : rs1;
    // Multiplexor ALUBsrc
  assign ALUBSrc_result = (ALUBSrc) ? imm : rs2;

    alu alu_inst (
        .operand1(ALUASrc_result),
        .operand2(ALUBSrc_result),
        .alu_control(ALUOpcode),
        .result(resultado_alu),
        .zero()
    );

    data_memory dm_inst (
        .clk(clk),
        .mem_write(DMWr),
        .mem_ctrl(DMCtrl),
        .address(resultado_alu),
        .write_data(rs2),
        .read_data(dm_read_data)
    );
    assign pc_plus_4 = pc + 32'd4;
    assign next_pc = (branch) ? resultado_alu : pc_plus_4;
  	always_comb begin
    case (RUDataWrSrc)
        2'b00: ru_wr_src = resultado_alu;
        2'b01: ru_wr_src = dm_read_data;
        2'b10: ru_wr_src = pc_plus_4;
        default: ru_wr_src = 32'b0; // Valor por defecto en caso de un valor no esperado
    endcase
    end
    
    // Sumador que suma 4 al pc
    
    assign result = resultado_alu;
    assign rs1R = ALUASrc;
    assign rs2R = ALUBSrc;
    assign rs1_data = ALUASrc_result;
    assign rs2_data = ALUBSrc_result;
    assign datos_alRegistro = ru_wr_src;

endmodule
