`timescale 1ns / 1ps



// for all the ports present in our design,
// We will declare a logic type variable for all variables (following the same size)
 
interface ahb_if;
  
  logic clk;
  logic [31:0] hwdata;
  logic [31:0] haddr;
  logic [2:0] hsize;
  logic [2:0] hburst;
  logic hresetn, hsel, hwrite;
  logic [1:0] htrans;
  logic [1:0] hresp;
  logic hready;
  logic [31:0] hrdata;
  
  logic [31:0] next_addr;
  
  
endinterface
///////////////////////////////////////////////
 
 // transaction -> one ahb transfer 
class transaction; // to keep track of all i/p and o/p ports in our design
  // class used for easy randomization and copying
 rand bit [4:0] ulen; // burst length
 rand bit [31:0] hwdata; 
 rand bit [31:0] haddr; 
 rand bit [2:0]  hsize;  
 rand bit [2:0]  hburst; 
 // randomizing above variables, so that whenever we do transaction.randomize()
 // non randomizable variables, as they come from dut
  bit hresetn;
  rand bit hwrite;
  bit [1:0] htrans;
  bit [1:0] hresp;
  bit hready;
  bit [31:0] hrdata;
  
  constraint write_c {
    hwrite dist {1 :/ 1, 0:/ 1};
  } // equating probability of read and write operation doing 50:50
  
  constraint ulen_c {
    ulen == 5;
  } // burst length force kardiya to 5 
  
  // random constraints to check architecture
  constraint burst_c {
  hburst == 6;
  } // wrap 16 forced
  
  constraint addr_c {
  haddr == 5;
  }

 // deep copy 
  function transaction copy();
   copy = new(); // adding constructor to copy 
    copy.hwdata = this.hwdata;
    copy.haddr  = this.haddr;
    copy.hsize  = this.hsize;
    copy.hburst = this.hburst;
    copy.hwrite = this.hwrite;
    copy.htrans = this.htrans;
    copy.hresp  = this.hresp;
    copy.hready = this.hready;
    copy.hrdata = this.hrdata;
    copy.ulen   = this.ulen;
  endfunction
   
 endclass
 
 
//////////////////////////////////////////////////////
 
class generator;
 // to generate a random value for the variables where we have added a rand variable then send to driver
  
  transaction tr; // creating handler
  
 mailbox #(transaction) mbxgd; // to communicate data from generator to driver 
  
 mailbox #(bit [4:0]) mbxgm; // gen to monitor ( ulen that we want to work around)
  
  
  event done; // generation completed
  event drvnext; // when driver completes task
  event sconext; // when scoreboard completes task
 
   int count = 0; // to keep track of no. of transactions
   
  
 function new( mailbox #(transaction) mbxgd, mailbox #(bit[4:0]) mbxgm); // declaring constructor
    this.mbxgd = mbxgd; // to connect mbxgd
    this.mbxgm = mbxgm; // to connect to the one in generator class 
  tr =new(); // creating constructor 
  endfunction
  
  
  
    task run();
    // count -> holds number of transaction
   repeat(count) begin
    assert(tr.randomize) else $error("Randomization Failed"); // assert to check if randomize is successful
      $display("[GEN] : DATA SENT TO DRV");
    mbxgd.put(tr.copy); // sending deep copy
      mbxgm.put(tr.ulen);
    @(drvnext); // waiting for trigger from driver
    @(sconext); // waiting for trigger from scoreboard
    end
    
    ->done;
  endtask
  
   
endclass
///////////////////////////////////////////////////////
 
 
class driver;
  
  virtual ahb_if vif; // taking access to an interface 
 // as we need to trigger dut
 // allows for driver -> interface -> dut
  
  transaction tr; // to hold object received from gen
  
 event drvnext; // generator waits for this before sending next transaction, basically to synchronize 
  
  mailbox #(transaction) mbxgd;
 
  
  function new( mailbox #(transaction) mbxgd );
    this.mbxgd = mbxgd; 
  endfunction
  
  task reset();
    vif.hresetn <= 1'b0;
    vif.hwdata  <= 0;
    vif.haddr   <= 0; 
    vif.hsize   <= 0;
    vif.hwrite  <= 0;
    vif.hsel    <= 0;
    vif.htrans  <= 0;
    repeat(10) @(posedge vif.clk);
    vif.hresetn <= 1'b1;
    $display("[DRV] : RESET DONE");
  endtask
  
  /////////////////////////////single transfer write
  
  task single_tr_wr();
    
   @(posedge vif.clk);
    
   vif.hresetn <= 1'b1; // ensuring reset inactive
    
   vif.hburst <= 3'b000;
   
   vif.hwrite <= 1'b1;
   vif.hsel   <= 1'b1;
    
   vif.hwdata <= $urandom_range(1,50);
   vif.haddr  <= tr.haddr;
   vif.hsize  <= 3'b010; // word transfer 
   
   vif.htrans <= 2'b10; // nonseq, as first transfer
   
   @(posedge vif.hready);
   @(posedge vif.clk);
   ->drvnext; // notifying generator that driver is finished
   $display("[DRV] : SINGLE TRANSFER WRITE ADDR : %0d DATA : %0d", tr.haddr, vif.hwdata); 
  
  
  endtask
  
  ///////////////////////////////Single transfer read
 task single_tr_rd();
    
   @(posedge vif.clk);
    
   vif.hresetn <= 1'b1;
    
   vif.hburst <= 3'b000;
   
   vif.hwrite <= 1'b0;
   vif.hsel   <= 1'b1;
    
   vif.hwdata <= 0;
   vif.haddr  <= tr.haddr; // address comes from transaction object 
   vif.hsize  <= 3'b010; // word  
   
   vif.htrans <= 2'b00;
   
   @(posedge vif.hready);
   @(posedge vif.clk);
   ->drvnext; 
   $display("[DRV] : SINGLE READ TRANSFER ADDR : %0d DATA : %0d", tr.haddr, vif.hwdata); 
   
  endtask
  
assign vif.next_addr = dut.next_addr;
 
endmodule
