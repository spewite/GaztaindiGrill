# Arquitectura del Firmware - GaztaindiGrill

Este documento describe la arquitectura del software para el microcontrolador ESP32 del proyecto GaztaindiGrill. Su objetivo es servir como guía de mantenimiento y referencia para futuras modificaciones.

## 1. Estructura de Clases y Separación de Responsabilidades

El firmware sigue un diseño orientado a objetos para separar las responsabilidades y facilitar su mantenimiento. Las clases principales son:

-   **`GaztaindiGrill.cpp` (Fichero Principal):**
    -   Es el punto de entrada de la aplicación (`setup()` y `loop()`).
    -   Gestiona las conexiones primarias (WiFi y MQTT).
    -   Actúa como un **despachador principal**: en el `loop()` llama a las actualizaciones de los sistemas principales y en el `callback` de MQTT enruta los mensajes a la clase `Grill` correspondiente.

-   **`GrillSystem`:**
    -   Clase contenedora que gestiona el conjunto de todas las parrillas (`Grill`). En nuestro caso, crea y mantiene las 2 instancias de `Grill`.

-   **`Grill`:**
    -   Representa una única parrilla. Actúa como una **fachada (Facade)** que simplifica la interacción con los subsistemas.
    -   Recibe los comandos MQTT (ej: "execute_program") y los delega al gestor apropiado (ej: `ProgramManager`).

-   **`ProgramManager`:**
    -   **Es el cerebro de la ejecución de programas.** Contiene la máquina de estados (`ProgramState`, `StepState`) que controla el avance de un programa paso a paso.
    -   Interpreta el JSON del programa y lo convierte en una secuencia de acciones.
    -   Mantiene el estado del programa actual **en RAM** (no sobrevive a reinicios en la versión actual).

-   **`MovementManager`:**
    -   Responsable de todo el movimiento físico: actuador lineal (posición) y motor de rotación (inclinación).

-   **`GrillMQTT`:**
    -   Es un "wrapper" o envoltorio que simplifica la comunicación con el broker MQTT. Centraliza la lógica para publicar y suscribirse a topics, parsearlos, etc.

-   **`HardwareManager`, `GrillSensor`, `StatusLed`:**
    -   Clases de bajo nivel que abstraen la interacción directa con el hardware (sensores, motores, LEDs).

## 2. Flujo de Sincronización de Estado (Multi-Usuario)

Este es el flujo clave para asegurar que cualquier cliente que se conecte pueda ver el estado actual de un programa en ejecución.

**Problema:** Un nuevo cliente (Usuario B) se conecta y necesita saber qué programa está ejecutando la parrilla, que fue iniciado por el Usuario A. No puede pedirlo a la API, ya que la API podría tener una versión más nueva del programa.

**Solución (Arquitectura de Petición y Respuesta):**

1.  **Estado Ligero Periódico:** El ESP32 publica constantemente (cada pocos segundos) un mensaje **ligero** en el topic `grill/{id}/program_status_response`. Este mensaje solo contiene el progreso:
    ```json
    { "isRunning": true, "programId": 123, "currentStepIndex": 2, "elapsedTime": 45 }
    ```

2.  **Petición del Nuevo Cliente:** Un cliente nuevo recibe este mensaje. Detecta que el programa `123` está en ejecución, pero no tiene sus detalles (nombre, pasos, etc.). Para obtenerlos, publica un mensaje de **petición de un solo uso**:
    -   **Topic:** `grill/{id}/get_running_program_details`
    -   **Payload:** `{}` (vacío)

3.  **Respuesta desde la Fuente de Verdad:** El ESP32 (específicamente la clase `Grill`) está escuchando ese topic. Al recibir la petición:
    -   Le pide al `ProgramManager` el JSON completo del programa que tiene guardado en su memoria RAM.
    -   Publica este JSON completo como **respuesta de un solo uso** en el topic:
        -   `grill/{id}/running_program_details_response`

4.  **Sincronización del Cliente:** El nuevo cliente recibe esta respuesta, guarda los detalles del programa en su caché local y, a partir de ese momento, solo necesita los mensajes de estado ligeros para actualizar la interfaz.

## 3. Gestión de Desconexiones (Robustez del Sistema)

Para evitar "estados fantasma" en los clientes (que la UI muestre un programa en ejecución cuando el ESP32 está apagado), se utiliza el mecanismo **Last Will and Testament (LWT)** de MQTT.

-   **Al Conectar:** El ESP32 se registra en el broker con una "última voluntad": si se desconecta de forma inesperada, el broker debe publicar `offline` en el topic `grill/{id}/status`.
-   **Al Desconectar:** El broker ejecuta la "voluntad" y notifica a todos los clientes que la parrilla está desconectada.
-   **En el Cliente:** La aplicación web está suscrita a `grill/+/status`. Si recibe `offline`, actualiza la interfaz para mostrar "Parrilla Desconectada" y limpia cualquier estado de programa en ejecución para esa parrilla.

## 4. Nota sobre la Persistencia de Estado (Futuras Mejoras)

**IMPORTANTE:** La versión actual del firmware **NO guarda el estado de ejecución si el ESP32 se reinicia o se corta la alimentación.**

-   **Decisión de Diseño:** Se eliminó la lógica de guardado en la memoria Flash interna para simplificar el código y evitar el desgaste prematuro del chip por escrituras constantes.

-   **Solución Futura (Ideal):** Para implementar una reanudación precisa al segundo, la solución correcta es añadir al circuito un chip de **memoria FRAM (RAM Ferroeléctrica)**.
    -   La FRAM permite escrituras constantes y rápidas sin desgaste y no es volátil.
    -   La arquitectura futura consistiría en guardar el JSON del programa en la Flash una vez, y actualizar el progreso (`paso_actual`, `segundos_transcurridos`) en la FRAM cada segundo.
