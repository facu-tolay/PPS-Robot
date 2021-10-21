Cosas para mejorar el código:
- Implementar manejo de erores, hasta el momento en ningun lado se compruba el retorno de las funciones.
- Ser menos repetitivos en la inicializacion en app main.

Idea:
- Es implementar una cola de mensaje por tarea y reemplazar el actual modelo de cola por elemento. Para eso debemos ser capaz de atender multiples fuentes de una sola cola. Esto es sobre todo por la cantidad de elementos (4 motores y 5 sensores), no tiene sentido que haya 9 colas.
- En realidad habia 4 colas, pero ahora como tenemos mas sensores que motores debemos separarlas. 
