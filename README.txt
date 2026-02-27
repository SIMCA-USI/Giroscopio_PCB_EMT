La carpeta de "Datos" tiene los codigos que se han utilizado para la detección de
las vibraciones.

Como os he explicado están las versiones de:
- logger_plot.py
- logger_plot2.py
- logger_plot3.py
- logger_plot4.py
Cada una es una versión mejorada o con añadidos de la anterior.

Luego hay más códigos que no son necesarios para la recolección de datos pero 
os los explico:
- graficas.py (genera las gráficas de los csv que se han grabado)
- arreglar_csv_mpu.py (aumentar los datos de las grabaciones que hicimos en cocheras)
Este último lo he usado solo para comprobar que el problema era ese (lo era)
- Correlacion.py y analisis_vibraciones.py (son códigos que hice a raíz de los datos que se recogen en el
pdf que tenemos, no lo vamos a usar porque no me convence mucho y porque lo haremos con 
algún programa vuestro)
- ruta.py (es un código que devuelve un .html de la ruta que hemos hecho pintando la línea de distintos
colores dependiendo de unos rangos de velocidad)


La carpeta "vibraciones_EMT" es lo que os he comentado de la PCB (el aparatito que usamos). Dentro hay muchas cosas
pero las que más nos interesan son:

-platformio.ini (que es lo que usamos para ponerle el nombre de la ubicacion)

y dentro de la carpeta "src" el codigo:

- main.cpp (que tiene toda la lógica del uso de la PCB)
