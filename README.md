# MREK2-walk

Ходьба **одной ноги** (3 привода) на CANopen.  
База: актуальный **MREK2-master** (кинематика, Fourier, пресеты, Walk).

```
Nucleo-F767ZI → UART PE0/PE1 → UART–CAN → CAN → 3× ELD2
```

## Что взято из master

| Параметр | Значение |
|----------|----------|
| `limbSegment` | `length, baseAngle, zeroLength, L1, L2` |
| hip | `0.42, 148.22, 275.07, 150.97, 135` |
| knee | `0.45, 125, 238.08, 183, 80` |
| foot | `0, 90, 199.72, 183, 80` |
| Fourier / scale | как `fourierTrajectoryA2` + `Q3Left` |
| Пресеты моторов | `TRAJECTORY_PRESETS` (оси 0–2) |
| После выхода на траекторию | `delay(3000)` + `True` |

## Node-ID

| Ось | ID (default) |
|-----|----------------|
| hip | 1 |
| knee | 2 |
| foot | 3 |

`NID_*` в `controlling.h`.

## Serial

```
Walk:10;10;0;12;5;1
# scale_x10; scale_z10; from_current; period; cycles; hold

# или полный формат master:
Walk:1;1;0;10;10;0;12;5;1
```

`Stop` / `Start` / `Error`

### Телеметрия

```
True
<ANGLES:hip,knee,foot>
<LOAD:I_h,I_k,I_f,T_h,T_k,T_f>   # % номинала, TPDO2
...
<END>
```

На ELD2 ток часто 0 — смотрите момент **T**.

## Сборка

```bash
pio run
pio run -t upload
```
