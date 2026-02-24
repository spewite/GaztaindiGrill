# Propuesta de Arquitectura: Sistema de Modos Dual y Single (GaztaindiGrill)

Este documento resume la sesión de análisis y refactorización del sistema de movimiento para las parrillas GaztaindiGrill, enfocándose en la implementación robusta del manejo **Single** (individual) y **Dual** (sincronizado).

## 1. El Problema Identificado (Deadlock Lógico)

En la versión original, el sistema sufría de un bucle infinito de lógica que impedía el movimiento físico en modo Dual:

1.  **Comando MQTT:** El `Grill` recibía una orden de "Subir".
2.  **MovementManager::go_up():** Detectaba que el modo era `DUAL` y, en lugar de mover el motor, solo seteaba `modeManager->dual_direction = UPWARDS`.
3.  **DualModeCoordinator::update():** Veía la dirección `UPWARDS` y llamaba a `grill0->go_up()` y `grill1->go_up()`.
4.  **Bucle:** `go_up()` volvía a ver que el modo era `DUAL` y volvía a setear la variable sin dar energía al motor. **Resultado:** Los motores nunca se movían.

Además, el sensor `is_at_top()` devolvía una variable lógica en modo Dual, impidiendo que el reseteo de sincronización detectara el tope físico real.

---

## 2. La Solución: Separación de "Intención" y "Ejecución"

Para solucionar esto siguiendo las mejores prácticas (SRP - Single Responsibility Principle), se ha propuesto dividir los métodos de movimiento en dos niveles:

### A. Nivel de Ejecución (Músculo/Raw)
Métodos que dan energía al motor directamente sin hacer preguntas sobre el modo.
*   `move_up_raw()`
*   `move_down_raw()`
*   `stop_raw()`

**¿Quién los usa?** 
- El `DualModeCoordinator` para mover ambas parrillas a la vez.
- El propio `MovementManager` cuando el modo es `SINGLE`.

### B. Nivel de Intención (Cerebro/Inteligente)
Métodos que deciden qué hacer basándose en el modo actual.
*   `go_up()`
*   `go_down()`
*   `stop_lineal_actuator()`

**Lógica interna sugerida:**
```cpp
void MovementManager::go_up() {
    if (modeManager->mode == SINGLE) {
        move_up_raw(); // Actuación local
    } else {
        modeManager->dual_direction = UPWARDS; // Petición al Coordinador
    }
}
```

---

## 3. Flujo de Control Corregido

### Movimientos Automáticos (`go_to`)
Gracias a esta estructura, funciones como `go_to(posicion)` o `go_to_temp(grados)` funcionan automáticamente en modo Dual sin cambiar su código interno:
1.  `go_to` llama a `go_up()`.
2.  `go_up()` pone la dirección en el `ModeManager`.
3.  El `DualModeCoordinator` mueve ambos motores usando los métodos `_raw`.
4.  Cuando la parrilla que recibió la orden llega a su destino, su `handle_stop` llama a `stop_lineal_actuator()`, lo que pone el estado en `STILL` y detiene a ambas.

### Sincronización de Sensores
El `GrillSensor` debe ser siempre "sincero":
*   `is_at_top()` debe devolver siempre el estado del **limit switch físico**.
*   Esto permite que el `DualModeCoordinator` realice la secuencia de reseteo correctamente (subir ambas hasta que los interruptores físicos se activen).

---

## 4. Plan de Acción (Refactorización Quirúrgica)

1.  **MovementManager.h/.cpp**: 
    *   Re-integrar el puntero a `ModeManager`.
    *   Implementar métodos `_raw` para control directo del hardware.
    *   Hacer que `go_up/down/stop` sean los selectores de modo.
2.  **GrillSensor.cpp**: Asegurar que `is_at_top()` solo dependa del hardware.
3.  **DualModeCoordinator.cpp**: Cambiar todas las llamadas a los métodos `_raw` de las parrillas.
4.  **GrillSystem.cpp**: Implementar `set_system_mode` para gestionar transiciones seguras entre modos.

---
*Documentación generada durante la sesión de consultoría técnica con Gemini CLI - 23 de febrero de 2026.*
