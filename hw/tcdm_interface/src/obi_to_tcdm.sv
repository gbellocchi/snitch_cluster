// Copyright 2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Author: Lucia Luzi <luzil@ethz.ch>
// Author: Gianluca Bellocchi <gianluca.bellocchi@unimore.it>

`include "reqrsp_interface/typedef.svh"

/// Convert OBI to TCDM protocol.
module obi_to_tcdm #(
    parameter type obi_req_t = logic,
    parameter type obi_rsp_t = logic,
    parameter type tcdm_req_t = logic,
    parameter type tcdm_rsp_t = logic,
    parameter int unsigned AddrWidth   = 0,
    parameter int unsigned DataWidth   = 0,
    parameter int unsigned IdWidth     = 0,
    parameter int unsigned UserWidth   = 0,
    parameter int unsigned MemRespLat  = 0,
    parameter int unsigned NumChannels = 1
) (
    input  logic      clk_i,
    input  logic      rst_ni,
    input  obi_req_t  [NumChannels-1:0] obi_req_i,
    output obi_rsp_t  [NumChannels-1:0] obi_rsp_o,
    output tcdm_req_t [NumChannels-1:0] tcdm_req_o,
    input  tcdm_rsp_t [NumChannels-1:0] tcdm_rsp_i
);

  typedef logic [AddrWidth-1:0]   addr_t;
  typedef logic [DataWidth-1:0]   data_t;
  typedef logic [DataWidth/8-1:0] strb_t;
  typedef logic [UserWidth-1:0]   user_t;
  typedef logic [IdWidth-1:0]     id_t;

  `REQRSP_TYPEDEF_ALL(reqrsp, addr_t, data_t, strb_t, user_t)

  for (genvar i = 0; i < NumChannels; i++) begin : gen_tcdm_obi_adapt
    // Backpressure on new requests, while another transaction is still not consumed.
    logic can_accept;
    // Merged read-write R-channel responses after pipeline.
    logic  obi_p_rvalid;
    data_t obi_p_rdata;
    id_t   obi_p_rid;
    /// Hold register signals
    logic  r_pending;
    data_t r_data_reg;
    id_t   r_id_reg;

    assign tcdm_req_o[i].q_valid = obi_req_i[i].req & can_accept;
    assign tcdm_req_o[i].q = '{
      addr:  obi_req_i[i].a.addr,
      write: obi_req_i[i].a.we,
      amo:   snitch_pkg::AMONone,
      data:  obi_req_i[i].a.wdata,
      strb:  obi_req_i[i].a.be,
      user:  '0
    };
    assign obi_rsp_o[i].gnt = tcdm_rsp_i[i].q_ready & can_accept;

    /// This pipeline drives backward TCDM write acknowledgement, which would otherwise be missing 
    /// as TCDM writes are fire-and-forget. Instead, OBI has a R-channel response for both reads 
    /// and writes. The converter thus drives write ack after MemRespLat cycles from the first 
    /// grant to the OBI interface.
    if (MemRespLat > 0) begin : gen_id_pipeline
      // Pipelined signals
      logic [MemRespLat-1:0] p_ack;
      logic [MemRespLat-1:0] p_we;
      id_t  [MemRespLat-1:0] p_id;

      /// R-channel response pipeline.
      always_ff @(posedge clk_i or negedge rst_ni) begin: r_resp_pipeline
        if (!rst_ni) begin
          p_ack <= '0;
          p_we <= '0;
          p_id <= '0;
        end else begin
          // Load current ack.
          p_ack[0] <= obi_req_i[i].req & obi_rsp_o[i].gnt;
          p_we[0] <= (obi_req_i[i].req & obi_rsp_o[i].gnt) ? obi_req_i[i].a.we  : '0;
          p_id[0] <= (obi_req_i[i].req & obi_rsp_o[i].gnt) ? obi_req_i[i].a.aid : '0;
          // Implement pipeline as a shift register.
          for (int k = 1; k < MemRespLat; k++) begin
            p_ack[k] <= p_ack[k-1];
            p_we[k] <= p_we[k-1];
            p_id[k] <= p_id[k-1];
          end
        end
      end

      /// Prepare data for the OBI R-channel.
      assign obi_p_rvalid = tcdm_rsp_i[i].p_valid |
                          (p_ack[MemRespLat-1] & p_we[MemRespLat-1]);
      assign obi_p_rdata  = tcdm_rsp_i[i].p.data;
      assign obi_p_rid    = p_id[MemRespLat-1];

      // Prevent new grants while the hold register is occupied.
      assign can_accept = ~r_pending & ~(obi_p_rvalid & ~obi_req_i[i].rready);

      // Hold register: used to absorb an R response when the OBI interface cannot accept it.
      always_ff @(posedge clk_i or negedge rst_ni) begin: r_hold_reg
        if (!rst_ni) begin
          r_pending <= '0;
          r_data_reg <= '0;
          r_id_reg <= '0;
        end else begin
          if (r_pending) begin
            if (obi_req_i[i].rready) r_pending <= '0;
          end else if (obi_p_rvalid && !obi_req_i[i].rready) begin
            r_pending <= '1;
            r_data_reg <= obi_p_rdata;
            r_id_reg <= obi_p_rid;
          end
        end
      end

      // Assign local R response to OBI response interface.
      assign obi_rsp_o[i].rvalid = r_pending | obi_p_rvalid;
      assign obi_rsp_o[i].r = '{
        rdata:      r_pending ? r_data_reg : obi_p_rdata,
        rid:        r_pending ? r_id_reg   : obi_p_rid,
        err:        1'b0,
        r_optional: '0
      };

    // Zero-latency: write response is valid in the same cycle as the grant.
    end else begin : gen_id_zero_latency
      /// Prepare data for the OBI R-channel.
      assign obi_p_rvalid = tcdm_rsp_i[i].p_valid |
                            (obi_req_i[i].req & obi_rsp_o[i].gnt & obi_req_i[i].a.we);
      assign obi_p_rdata  = tcdm_rsp_i[i].p.data;
      assign obi_p_rid    = obi_req_i[i].a.aid;

      // Prevent new grants while the hold register is occupied.
      assign can_accept = ~r_pending;

      // Hold register: used to absorb an R response when the OBI interface cannot accept it.
      always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
          r_pending <= '0;
          r_data_reg <= '0;
          r_id_reg <= '0;
        end else begin
          if (r_pending) begin
            if (obi_req_i[i].rready) r_pending <= '0;
          end else if (obi_p_rvalid && !obi_req_i[i].rready) begin
            r_pending <= '1;
            r_data_reg <= obi_p_rdata;
            r_id_reg <= obi_p_rid;
          end
        end
      end

      // Assign local R response to OBI response interface.
      assign obi_rsp_o[i].rvalid = r_pending | obi_p_rvalid;
      assign obi_rsp_o[i].r = '{
        rdata:      r_pending ? r_data_reg: obi_p_rdata,
        rid:        r_pending ? r_id_reg  : obi_p_rid,
        err:        1'b0,
        r_optional: '0
      };
    end

  end

endmodule
