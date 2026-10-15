# How the drone follows the pink LED

High-level only — what the drone does, not how the code does it.

- **`led-tracking-loops.drawio`** — open in [diagrams.net](https://app.diagrams.net)
  or the VS Code Draw.io Integration extension. Editable.
- **below** — the same thing as Mermaid. draw.io imports it via
  *Arrange → Insert → Advanced → Mermaid*; GitHub renders it inline.

```mermaid
flowchart TB
  A([Take off])
  B[Hover in place and turn slowly,<br>looking for the pink LED]
  C{Pink LED<br>spotted?}
  D([Give up and land])

  subgraph LOOP["LED centring loop"]
    E[See how far the LED is<br>from the middle of the view]
    F{Is it in<br>the middle?}
    G[Turn left or right<br>toward it]
    H[Fly forward slowly]
    I{Can we still<br>see it?}
    J[Stop and hover.<br>Wait for it to come back]
  end

  A --> B --> C
  C -->|no, after a while| D
  C -->|yes| E
  E --> F
  F -->|not yet| G
  G -->|look again| E
  F -->|yes| H
  H --> I
  I -->|yes, keep going| E
  I -->|no| J
  J -.->|it comes back| E

  classDef act fill:#dae8fc,stroke:#6c8ebf,color:#000
  classDef ask fill:#e1d5e7,stroke:#9673a6,color:#000
  classDef end1 fill:#d5e8d4,stroke:#82b366,color:#000
  classDef stop fill:#f8cecc,stroke:#b85450,color:#000
  class B,E,G,H,J act
  class C,F,I ask
  class A end1
  class D stop
```

Three things worth knowing:

**Height never changes.** It is set at take-off and held for the whole flight.
Only turning and flying forward are controlled — the drone never climbs or
descends to follow the LED, even if the LED is well above or below it.

**The camera only tells us which way the LED is, never how far.** Nothing here
knows when it has arrived, so nothing stops it short of the target.

**Staying level and steady is not on this page.** The flight controller handles
that on its own, continuously, whatever the LED behaviour is doing.
