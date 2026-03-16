`timescale 1ns / 1ps
 
`define NON_SEQ     2'd0
`define SEQ 	    2'd1
`define BUSY 	    2'd2
`define IDLE        2'd3
 
`define OKAY  2'b00
`define ERROR 2'b01
`define RETRY 2'b10
`define SPLIT 2'b11 // defines used to make the code readable and maintainable, so we don't have to change everywhere
 
 
module ahb_slave(
input clk,
input [31:0] hwdata,
input [31:0] haddr, 
input [2:0] hsize,
input [2:0] hburst,
input hresetn, hsel, hwrite,
input [1:0] htrans,
output reg [1:0] hresp,
output reg hready,
output reg [31:0] hrdata
);
 
 
 
reg [7:0] mem[256] = '{default : 12};
 
//////////
 function bit[31:0] single_tr (input bit [31:0] addr, input bit [2:0] hsize); // single transfer function
  // function performs only one single write transaction
  unique case(hsize) //// to enforce only one match exists
    3'b000: begin // case for byte transfer 8 bits
         mem[addr]  = hwdata[7:0];   
    end 
    
    3'b001: begin // halfword transfer 16 bits
         mem[addr]     = hwdata[7:0];
         mem[addr + 1] = hwdata[15:8];
    end
    
    3'b010: begin // word transfer 32 bits
     mem[addr]     = hwdata[7:0]; // 4 bytes written sequentially
         mem[addr + 1] = hwdata[15:8];
         mem[addr + 2] = hwdata[23:16];
     mem[addr + 3] = hwdata[31:24]; // little endian storage being followed as we store lsb first then msb's
    end
    endcase
    return addr;
    
endfunction
 
 
///////////////////////////////////////////////////////////////////////////////////
 function bit[31:0] unincr_wr (input bit [31:0] addr, input bit [2:0] hsize); // performs write and calculates next address // unspecified increment
  bit [31:0] raddr; // returns next address, as it is used for incrementing burst
  unique case(hsize) // to enforce only one match exists 
    
     3'b000: begin
          mem[addr]    = hwdata[7:0];
          raddr        = addr + 1; // next transfer one byte later
     end
     
     3'b001: begin
         mem[addr]     = hwdata[7:0];
         mem[addr + 1] = hwdata[15:8];
         raddr         = addr + 2; // next transfer 2 bytes later / halfword 16 bit also
     end
 
     3'b010: begin
         mem[addr]     = hwdata[7:0];
         mem[addr + 1] = hwdata[15:8];
         mem[addr + 2] = hwdata[23:16];
         mem[addr + 3] = hwdata[31:24];
         raddr         = addr + 4; // next transfer word later
     end
 endcase
    
return raddr; // returns the next address for the following transfers
 
endfunction
 
 
 
 
 
 // boundary limit is [7:0] because max case we can consider is wrap 16 with 4 byte transfer, which is 64, so 2^7 karke we can handle it
//////////////////////////////////////////////////////////////////////////////
 function bit[7:0] boundary(input bit [2:0] hburst, input [2:0] hsize); // wrap boundary calculation implementation -> address gets wrapped back when boundary is reached
  // eg: word transfer + wrap 4 
  // 12 is start address, 12 0 4 8/ gets wrapped at 16 to 0
   bit [7:0] temp;
  unique case(hsize) // boundary = transfer_length * size of transfer 
   3'b000: begin // hsize = 000 ( 1bit ) , 001 ( 2 bit ) , 010 (4 bit)
       unique case (hburst) // 010 wrap 4, 100 wrap 8, 110 wrap 16
              3'b010 : temp = 4 * 1; // 1 bit transfer with wrap 4 
              
              3'b100 : temp =  8 * 1; // case of 1 bit transfer with wrap 8 condition of h burst
              
              3'b110 : temp = 16 * 1; // case of 1 bit transfer with wrap 16 condition of hburst
            endcase
      end
      
      3'b001 : begin
            unique case (hburst)
              3'b010 :  temp = 4 * 2; // 2 bit transfer with wrap 4 condition
              
              3'b100 : temp =  8 * 2; // case for wrap 8 with 2 bit transfer
              
              3'b110 : temp = 16 * 2;
            endcase
      end
      
      
      3'b010 : begin
            unique case (hburst)
              3'b010 :  temp = 4 * 4; // word + wrap 4 -> 4 * 4 which implies boundary of 16
              
              3'b100 : temp =  8 * 4; // word + wrap8 -> boundary of 32 
              
              3'b110 : temp = 16 * 4;
            endcase
    end
    
    endcase
    
    return temp; // to store boundary case
    
endfunction
 
 // wrap function
function bit[31:0] wrap_wr (input bit [31:0] addr, input bit [7:0] boundary, input [2:0] hsize);
  bit [31:0] addr1, addr2, addr3,addr4;
  
  unique case(hsize)
    3'b000: begin
     mem[addr] = hwdata[7:0]; // case for byte transfer
       
     if((addr + 1) % boundary == 0) // to check if we reached end of wrap boundary
      addr1 = (addr + 1) - boundary; // if yes, wrap back to start of the block
        else
         addr1 = (addr + 1);
         
       return addr1;  
    end
    
    3'b001: begin
        mem[addr] = hwdata[7:0];
       
        if((addr + 1) % boundary == 0)
         addr1 = (addr + 1) - boundary;
        else
         addr1 = (addr + 1);
         
     mem[addr1] = hwdata[15:8]; // ADDRESS WRAPPING CHECKED AFTER EACH BYTE INCREMENT
         
       if((addr1 + 1) % boundary == 0)
         addr2 = (addr1 + 1) - boundary;
        else
         addr2 = (addr1 + 1);
         
        return addr2;
        
       end
    
    3'b010: begin
        
        mem[addr] = hwdata[7:0];
       
        if((addr + 1) % boundary == 0)
         addr1 = (addr + 1) - boundary;
        else
         addr1 = (addr + 1);
         
         mem[addr1] = hwdata[15:8];
         
       if((addr1 + 1) % boundary == 0)
         addr2 = (addr1 + 1) - boundary;
        else
         addr2 = (addr1 + 1);
         
         mem[addr2] = hwdata[23:16];
        
         if((addr2 + 1) % boundary == 0)
         addr3 = (addr2 + 1) - boundary;
         else
         addr3 = (addr2 + 1); 
         
        mem[addr3] = hwdata[31:24];
        
        if((addr3 + 1) % boundary == 0)
         addr4 = (addr3 + 1) - boundary;
         else
         addr4 = (addr3 + 1); 
       
       return addr4;
    end
    
  
  endcase
  
endfunction
////////////////////////////////////////////////////////////////////////////////////////////
 
 
 function bit[31:0] incr_wr(input bit [31:0] addr, input bit [2:0] hsize); // incrementing write
    
  bit [31:0] raddr; // performs a write and calculates next address for increment bursts; USED FOR BURSTS LIKE INCR, INCR4, INCR8, INCR16
    
    unique case(hsize)
    
     3'b000: begin // byte transfer
          mem[addr]    = hwdata[7:0];
          raddr        = addr + 1; 
     end
     
     3'b001: begin
         mem[addr]     = hwdata[7:0];
         mem[addr + 1] = hwdata[15:8];
         raddr         = addr + 2;
     end
 
     3'b010: begin
         mem[addr]     = hwdata[7:0];
         mem[addr + 1] = hwdata[15:8];
         mem[addr + 2] = hwdata[23:16];
         mem[addr + 3] = hwdata[31:24];
         raddr         = addr + 4;
     end
 endcase
 
return raddr; // becomes next address for burst transfer
 
endfunction
 
////////////////////////////////////////////////////////////////////////////////
////////////////////////////single transfer read
 
 function bit[31:0] single_tr_rd (input bit [31:0] addr, input bit [2:0] hsize); // performs one read transfer
  // corresponds to HBURST = single
    unique case(hsize)
    3'b000: begin
         hrdata[7:0] = mem[addr];  
    end 
    
    3'b001: begin
         hrdata[7:0]  = mem[addr];
         hrdata[15:8] = mem[addr + 1];
    end
    
    3'b010: begin
         hrdata[7:0]   = mem[addr];
         hrdata[15:8]  = mem[addr + 1];
         hrdata[23:16] = mem[addr + 2];
     hrdata[31:24] = mem[addr + 3]; // little endian mapping, lsb comes first 
    end
    endcase
    return addr; // returns same address and doesnt advance it 
    
endfunction
///////////////////////////////////////////////////////
 
//////////////////////////////////////////////////////////////////////////////////
/////////////////////////////Read for unspec length -> or incrementing read bhi bol sakte hain
function bit[31:0] unincr_rd (input bit [31:0] addr, input bit [2:0] hsize);
 bit [31:0] raddr; // raddr is the next address in the burst
    unique case(hsize)
    // supports burst like INCR, INCR4, INCR8, INCR16
     3'b000: begin
          hrdata[7:0] = mem[addr];
          raddr        = addr + 1; 
     end
     
     3'b001: begin
         hrdata[7:0] = mem[addr];
         hrdata[15:8] = mem[addr + 1];
         raddr         = addr + 2;
     end
 
     3'b010: begin
         hrdata[7:0]   = mem[addr];
         hrdata[15:8]  = mem[addr + 1];
         hrdata[23:16] = mem[addr + 2];
         hrdata[31:24] = mem[addr + 3];
         raddr         = addr + 4;
     end
 endcase
    
return raddr;
 
endfunction
 
//////////////////////////////////////////////////////
/////////////////////////////              wrapping read           ////////////////////////////////////////////////////
function bit[31:0] wrap_rd (input bit [31:0] addr, input bit [7:0] boundary, input [2:0] hsize);
  bit [31:0] addr1, addr2, addr3,addr4;
  
  unique case(hsize)
    3'b000: begin
       hrdata[7:0] = mem[addr];
       
     if((addr + 1) % boundary == 0) // rest of the process is same as wrapping write case;
         addr1 = (addr + 1) - boundary;
        else
         addr1 = (addr + 1);
         
       return addr1;  
    end
    
    3'b001: begin
         hrdata[7:0] = mem[addr];
       
        if((addr + 1) % boundary == 0)
         addr1 = (addr + 1) - boundary;
        else
         addr1 = (addr + 1);
         
         hrdata[15:8] = mem[addr1];
         
       if((addr1 + 1) % boundary == 0)
         addr2 = (addr1 + 1) - boundary;
        else
         addr2 = (addr1 + 1);
         
        return addr2;
        
       end
    
    3'b010: begin
        
        hrdata[7:0] = mem[addr];
       
        if((addr + 1) % boundary == 0)
         addr1 = (addr + 1) - boundary;
        else
         addr1 = (addr + 1);
         
        hrdata[15:8] = mem[addr1];
         
       if((addr1 + 1) % boundary == 0)
         addr2 = (addr1 + 1) - boundary;
        else
         addr2 = (addr1 + 1);
         
         hrdata[23:16] = mem[addr2];
        
         if((addr2 + 1) % boundary == 0)
         addr3 = (addr2 + 1) - boundary;
         else
         addr3 = (addr2 + 1); 
         
        hrdata[31:24] = mem[addr3];
        
         if((addr3 + 1) % boundary == 0)
         addr4 = (addr3 + 1) - boundary;
         else
         addr4 = (addr3 + 1); 
       
       return addr4;
    end
    
  
  endcase
  
endfunction
 
/////////////////////////////////////////////////////////////////////////////////////////
 function bit[31:0] incr_rd(input bit [31:0] addr, input bit [2:0] hsize); // incrementing burst read
  // reads data from memory
  // increments address and returns next address
    
    bit [31:0] raddr;
    
    unique case(hsize)
    
     3'b000: begin
          hrdata[7:0]  = mem[addr];
          raddr        = addr + 1; 
     end
     
     3'b001: begin
         hrdata[7:0]  = mem[addr];
         hrdata[15:8] = mem[addr + 1];
         raddr         = addr + 2;
     end
 
     3'b010: begin
         hrdata[7:0]   = mem[addr];
         hrdata[15:8]  = mem[addr + 1];
         hrdata[23:16] = mem[addr + 2];
         hrdata[31:24] = mem[addr + 3];
         raddr         = addr + 4;
     end
 endcase
 
return raddr;
 
endfunction
 
 
//////////////////////////////////////////////////////////////////////////////////
 
 typedef enum  {idle = 0, check_mode = 1, write = 2, read = 3, addr_decode = 4} state_type; // to define our fsm has 5 states, 
 // these 5 are sequential whereas state_type state, next_state are combinational
state_type state, next_state; // these 2 are the state registers
 // next_state is calculated by combinational logic
 
///////////////////////////////////////////////////////////////////////////////////
 // fsm has 2 parts -> this part is the sequential part; state change happens synchronously
 always_ff@(posedge clk) // sequential always block
begin
 if(!hresetn) // will be sensitive to positive edge, active low
state <= idle;
else
state <= next_state;
end // simple reset lofic
///////////////////////////////////////////////////////////////////////////////////
 
 
 
integer len_count = 0; // keeps track of burst length
 // eg: 4 transactions, len_count stores how many we have completed
reg first = 0; // to track first transfer of burst
 reg [31:0] retaddr; // This stores the address returned by burst functions.
 // eg: retaddr = incr_wr(next_addr,hsize)
 reg [31:0] next_addr; // address fsm will use for next transaction // address fsm actually uses
 reg [7:0] wboundary; // 8 bit variable to store the boundary
 
 
 
 
/////////////////////////////////////////////////////////////////////////////////
 // this part of fsm happens combinationally, to compute the next state 
always_comb // blocks contains pure combinational logic.
begin
    case(state)
  
          idle : // assigns all variables in our fsm to zero during idle state
          begin
          next_state = check_mode; // as soon as slave enters IDLE, it immediately prepares to check for transaction
          hready = 1'b0; // in idle slave isn't ready for transaction
          len_count = 0; // previous burst finished so we reset
          first = 0;
          hresp = `OKAY; // normal transfer
          end
  
          check_mode : // identifying whether we have a valid transaction request or not
          begin
                            hready = 1'b0; // again slave isnt ready yet 
           if(hresetn && hsel && hwrite) // first thing is reset should be high and hsel should be 1, now in this case hwrite is 1
            // depending on hwrite we decide whether we want to write data to memory or read from it
                                begin // VALID WRITE REQUEST
                                 if(haddr < 256) // checking if address is within the range
                                      begin
                                      next_state = addr_decode;
                                      end
                                     else // address is outside the range
                                      begin 
                                       next_state = idle;
                                       hresp = `ERROR;
                                      end
                                end
           else if (hresetn && hsel && !hwrite) // in this case now hwrite is 0 
            // basically logic for read operation
                                begin
                                        if(haddr < 256)
                                        begin  
                                        next_state = addr_decode;
                                        end
                                        else
                                        begin
                                        next_state = idle;
                                        hresp = `ERROR; // in this case hready is still 1, slave is responding with an error
                                        end
                                end
                           else // either reset has hit or hsel isnt 1
                                begin
                                next_state = idle;
                                                                end  
          end
  
         addr_decode: // to compute the address for the next transaction
          // and to determine whether we are reading or writing
          begin
           if(htrans == `NON_SEQ) // the beginning of the burst or first transaction of any transfer
                             begin
                                       next_addr = haddr; // next address should be whatever we have on the bus
                                       
                                       if(hwrite)
                                       next_state = write; // jumping to write state
                                       else
                                       next_state = read; // jumping to read state
                                       
                                       
                             end 
           // for all transactions after the first one
           else if (htrans == `SEQ)  // continuation of a burst, only first address comes from master
            // following address returned by the function
                             begin
                              next_addr  = retaddr; // next address becomes address returned by the function eg: incr_wr()
                                            
                                           if(hwrite)
                                           next_state = write;
                                           else
                                           next_state = read;
 
                            end
         
         
         end
  
  
         write: // writing data to memory
          begin
               case(hburst)
  
  //////////////////////////           Single Write at HADDR
                    3'b000: begin  ////single transfer
                     retaddr = single_tr(next_addr,hsize); // we call the single transfer function, while passing the 2 arguments
                     // not really used in this case as single write we return the same address itself.
                     // i.e. next_addr and hsize
                       hready     = 1'b1; // once transaction completes
                       next_state = idle; // jumping back to idle state
                       hresp = `OKAY;
                    end
   
   /////////////////////      INCREMENT for UNSPECIFIED LENGTH         
            
                   3'b001: 
                       begin   ////incr mode 
                       // unspecified length burst
                        // length isnt fixed
                       hready = 1'b1;
                        retaddr = unincr_wr(next_addr, hsize); // unspecified increment write
                        // does 2 things: 1) writes data into memory 2) computes next address
                        // retaddr is the next incremented address
                       hresp = `OKAY;      
                       
                       
                        if(len_count < 32) // limitation
                         // theoretically infinite so to curb it 
                         // assuming that master keeps sending seq transfers
                          begin
                          len_count = len_count + 1;
                          next_state = check_mode; 
                          end
                       else
                          begin
                          len_count = 0;
                          next_state = idle;
                          end   
                                                
                    end
 ////////////////////////////4 beat wrapping
 
 // WRAP 4 BURST
                // 4 transfers in a burst 
                // addresses wrap inside a boundary
                  3'b010: 
                        begin
                          
                          hready = 1'b1; // write completed, slave ready for next beat
                         wboundary = boundary(hburst, hsize); // function to calculate boundary size
                         retaddr   = wrap_wr(next_addr, wboundary, hsize); // writing data to memory
                         // computing next wrapped address
                         // inside wrap_wr = addr + transfer_size
                          hresp = `OKAY;
                                
                               
                                 if(len_count <= 2) // 0 1 2 3
                                  // after 4th beat, the transfer ends
                                  // counter starts at 0, so 3 more allowed after it
                                     begin
                                     len_count = len_count + 1;
                                     next_state = check_mode;
                                     end
                                     else
                                     begin
                                     next_state = idle;
                                     len_count = 0;
                                     end
                      end 
        
///////////////////////////////4 beat Incrementing
 
                 3'b011:
                               begin
                               
                                   hready = 1'b1;
                                   retaddr = incr_wr(next_addr, hsize);
                                   hresp = `OKAY;
                                   
                                   if(len_count <= 2)
                                     begin
                                     len_count = len_count + 1;
                                     next_state = check_mode;
                                     end
                                     else
                                     begin
                                     next_state = idle;
                                     len_count = 0;
                                     first = 0; 
                                     end
                                   
                               
                               end    
                               
////////////////////////////////////////////////8 beat wrapping
 
 
                  3'b100: 
                      begin
                                          
                                          hready = 1'b1; 
                                          wboundary = boundary(hburst, hsize);
                                          retaddr = wrap_wr(next_addr, wboundary, hsize);
                                          hresp = `OKAY;
                                           
                                   if(len_count <= 6)
                                    // 0 1 2 3 4 5 6 7
                                    // 8 transfers in the wrap
                                     begin
                                     len_count = len_count + 1;
                                     next_state = check_mode;
                                     end
                                     else
                                     begin
                                     next_state = idle;
                                     len_count = 0;
                                     end
                                    end
                                       
 ////////////////////////////////////////////////////////////////////////////////////////////////
 /////////////////////////////////////////8 beat Incrementing
                        3'b101:
                               begin
                                   hready = 1'b1;
                                   retaddr = incr_wr(next_addr, hsize);
                                   hresp = `OKAY;
                                   
                                 if(len_count <= 6)
                                     begin
                                     len_count = len_count + 1;
                                     next_state = check_mode;
                                     end
                                     else
                                     begin
                                     next_state = idle;
                                     len_count = 0;
                                     end
                               
                               end  
 
 /////////////////////////////////////////////////////////////////////////////////////////////
 /////////////////////////////////////////////////16 beat wrapping
 
                       3'b110: 
                          begin
                              
                              hready = 1'b1; 
                              wboundary = boundary(hburst, hsize);
                              retaddr = wrap_wr(next_addr, wboundary, hsize);
                              hresp = `OKAY;
                               
                                 if(len_count <= 14)
                                     begin
                                     len_count = len_count + 1;
                                     next_state = check_mode;
                                     end
                                     else
                                     begin
                                     next_state = idle;
                                     len_count = 0;
                                     end
                        end
 
 /////////////////////////////////////////////////////////////////////////////////
 //////////////////////16 beat incr
                            3'b111:
                               begin
                                   hready = 1'b1;
                                   retaddr = incr_wr(next_addr, hsize);
                                   hresp = `OKAY;
                                   
                                 if(len_count <= 14)
                                     begin
                                     len_count = len_count + 1;
                                     next_state = check_mode;
                                     end
                                     else
                                     begin
                                     next_state = idle;
                                     len_count = 0;
                                     end
                               
                                     end 
       
                 endcase
  end
 
read : begin
 
               case(hburst)
  
  //////////////////////////Single Write at HADDR
                    3'b000: begin  ////single transfer
                       retaddr = single_tr_rd(haddr,hsize);
                       hready     = 1'b1;
                       next_state = idle;
                       hresp = `OKAY;
                    end
   
   /////////////////////INCREMENT for UNSPECIFIED LENGTH         
            
                   3'b001: 
                       begin   ////incr mode
                       hready = 1'b1;
                       retaddr = unincr_rd(next_addr, hsize);
                       hresp = `OKAY;
        
                              if( len_count < 32) 
                              begin
                              len_count = len_count + 1;
                              next_state = check_mode;
                              end
                              else
                              begin
                              len_count = 0;  
                              next_state = idle;
                              end                            
                       end  
 ////////////////////////////4 beat wrapping
 
 
                  3'b010: 
                        begin
                          
                          hready = 1'b1; 
                          wboundary = boundary(hburst, hsize);
                          retaddr = wrap_rd(next_addr, wboundary, hsize);
                          hresp = `OKAY;
                          
                                   if(len_count <= 2)
                                     begin
                                     len_count = len_count + 1;
                                     next_state = check_mode;
                                     end
                                     else
                                     begin
                                     next_state = idle;
                                     len_count = 0;
                                     end 
                           
                            
                        end 
        
///////////////////////////////4 beat Incrementing read
 
                 3'b011:
                               begin
                                   hready = 1'b1;
                                   retaddr = incr_rd(next_addr, hsize);
                                   hresp = `OKAY;
                                   
                                 if(len_count <= 2)
                                     begin
                                     len_count = len_count + 1;
                                     next_state = check_mode;
                                     end
                                     else
                                     begin
                                     next_state = idle;
                                     len_count = 0;
                                     end
                               
                               end    
                               
////////////////////////////////////////////////8 beat wrapping
 
 
                  3'b100: 
                      begin
                                          
                                          hready = 1'b1; 
                                          wboundary = boundary(hburst, hsize);
                                          retaddr = wrap_rd(next_addr, wboundary, hsize);
                                          hresp = `OKAY;
                                           
                                  if(len_count <= 6)
                                     begin
                                     len_count = len_count + 1;
                                     next_state = check_mode;
                                     end
                                     else
                                     begin
                                     next_state = idle;
                                     len_count = 0;
                                     end
                                     
                                     
                                    end
                                       
 ////////////////////////////////////////////////////////////////////////////////////////////////
 /////////////////////////////////////////8 beat Incrementing
                        3'b101:
                               begin
                                   hready = 1'b1;
                                   retaddr = incr_rd(next_addr, hsize);
                                   hresp = `OKAY;
                                   
                                 if(len_count <= 6)
                                     begin
                                     len_count = len_count + 1;
                                     next_state = check_mode;
                                     end
                                     else
                                     begin
                                     next_state = idle;
                                     len_count = 0;
                                     end
                               
                               end  
 
 /////////////////////////////////////////////////////////////////////////////////////////////
 /////////////////////////////////////////////////16 beat wrapping
 
                       3'b110: 
                          begin
                              
                              hready = 1'b1; 
                              wboundary = boundary(hburst, hsize);
                              retaddr = wrap_rd(next_addr, wboundary, hsize);
                              hresp = `OKAY;
                               
                                  if(len_count <= 14)
                                     begin
                                     len_count = len_count + 1;
                                     next_state = check_mode;
                                     end
                                     else
                                     begin
                                     next_state = idle;
                                     len_count = 0;
                                     end
                        end
 
 /////////////////////////////////////////////////////////////////////////////////
 //////////////////////
                 // 16 beat incr
                            3'b111:
                               begin
                                   hready = 1'b1;
                                   retaddr = incr_rd(next_addr, hsize);
                                   hresp = `OKAY;
                                   
                                 if(len_count <= 14)
                                     begin
                                     len_count = len_count + 1;
                                     next_state = check_mode;
                                     end
                                     else
                                     begin
                                     next_state = idle;
                                     len_count = 0;
                                     end
                               
                                     end 
       
                 endcase
 
end
 
endcase
 
end
endmodule
