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
    parameter int unsigned BufDepth    = 1,
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

  // TCDM responds exactly BufDepth-1 cycles after a request is accepted.
  localparam int unsigned LatencyStages = BufDepth - 1;

  for (genvar i = 0; i < NumChannels; i++) begin : gen_tcdm_obi_adapt
    assign tcdm_req_o[i].q_valid = obi_req_i[i].req;
    assign tcdm_req_o[i].q = '{
      addr: obi_req_i[i].a.addr,
      write: obi_req_i[i].a.we,
      amo: reqrsp_pkg::AMONone,
      data: obi_req_i[i].a.wdata,
      strb: obi_req_i[i].a.be,
      user: '0
    };

    assign obi_rsp_o[i].gnt = tcdm_rsp_i[i].q_ready;

    // Propagate request ID and write flag. Stage 0 is loaded when a request is granted, while higher stages shift every cycle.
    // TCDM does not assert p_valid for write transactions, so rvalid for writes is derived from write_pipeline instead.
    if (LatencyStages > 0) begin : gen_id_pipeline
      id_t  [LatencyStages-1:0] id_pipeline;
      logic [LatencyStages-1:0] write_pipeline;

      always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
          id_pipeline    <= '0;
          write_pipeline <= '0;
        end else begin
          if (obi_req_i[i].req && obi_rsp_o[i].gnt) begin
            id_pipeline[0]    <= obi_req_i[i].a.aid;
            write_pipeline[0] <= obi_req_i[i].a.we;
          end
          for (int k = 1; k < LatencyStages; k++) begin
            id_pipeline[k]    <= id_pipeline[k-1];
            write_pipeline[k] <= write_pipeline[k-1];
          end
        end
      end

      assign obi_rsp_o[i].rvalid = tcdm_rsp_i[i].p_valid | write_pipeline[LatencyStages-1];

      assign obi_rsp_o[i].r = '{
        rdata:      tcdm_rsp_i[i].p.data,
        rid:        id_pipeline[LatencyStages-1],
        err:        1'b0,
        r_optional: '0
      };
    end else begin : gen_id_zero_latency
      // Zero-latency case: write response is valid in the same cycle as the grant; read response follows p_valid as usual.
      assign obi_rsp_o[i].rvalid = tcdm_rsp_i[i].p_valid |
                                   (obi_req_i[i].req & obi_rsp_o[i].gnt & obi_req_i[i].a.we);

      assign obi_rsp_o[i].r = '{
        rdata:      tcdm_rsp_i[i].p.data,
        rid:        obi_req_i[i].a.aid,
        err:        1'b0,
        r_optional: '0
      };
    end
  end

endmodule
