# CPM-4 Battery Management System (WIP)

<p align="center">
  <img src="media/cpm4.png" alt="CPM-4 BMS board" width="900">
</p>

This is a work in progress. The PCB is in design; firmware will follow. Details may change.

## Description
The **CPM-4** (Cell Protection Module, 4-series) supports **6–16 cells**. It incorporates automatic safety features and cell balancing. Set a few solder jumpers, wire it up, and get going quickly — or use advanced mode with dual **CAN bus** interfaces and a configuration app.

## Principle of Operation
The CPM-4 operates in **simple** or **advanced** mode.

- **Simple mode:** Boards ship with firmware and can be put into service without a smartphone or computer.  
  All state flags are **12 V** logic outputs that can be wired into the user’s system.  
  Solder jumpers let you set:
  - **Cell count**
  - **Voltage set-points (per cell)** — choose one profile:
    - *Lithium-ion:* Max-capacity **3.0–4.2 V** · Long-life **3.5–4.085 V**
    - *LiFePO₄:* Max-capacity **2.8–3.6 V** · Long-life **3.0–3.5 V**
  - **Cell chemistry:** Lithium-ion or **LiFePO₄**
  - **Role:** Master or Slave

- **Advanced mode:** Configure the module over the **CAN bus** with full control over thresholds, timing, and behaviour.

### Output pins
- Precharge  
- Contactor+  
- Contactor−  
- Aux Out 1 *(advanced mode only)*  
- Under-Temp  
- Over-Temp  
- Under-Voltage  
- Balancing  
- Over-Voltage  
- Charge Allow  
- Over-Current *(advanced mode only)*  
- Battery Cool  
- Battery Heat

### Input pins
- Charge Request  
- Fault In  
- Crash  
- Current Sensor *(advanced mode only)*  
- Aux In 1 *(advanced mode only)*  
- HV Present  
- **+ Four temperature-sensor inputs**

Two pins are reserved for the **high-voltage interlock loop (HVIL)** when building multi-board high-voltage systems.

---

- Follow progress via **Issues** and **Discussions** in this repo.  
- Licensing (intended): **Hardware** — CERN-OHL-W v2 • **Firmware** — Apache-2.0 • **Docs** — CC BY 4.0  
- **Safety:** High voltage is dangerous. For competent developers.
