# Guía de Github

Guía rápida para el **BIRRIA TEAM** para usar github desde la terminal. Esta guía es simplemente para poder llevar un workflow un poco más controlado, hacer gestión de versiones, etc. No contempla todos los aspectos de **Git**/**Github**.

## Antes que nada...
1. Tener [Git instalado](https://git-scm.com/downloads) en tu computadora.
2. Tener una cuenta de GitHub activa.
3. Que ya tengan acceso a este repositorio.

---

## 1. Descargar el proyecto (Solo se hace la primera vez)
Para descargar el código a tu computadora, abre la terminal en la carpeta donde quieras guardar el proyecto y ejecuta:
`bash
git clone https://github.com/Adrian-103/birria_team
`
*(Ojo: Para otros proyectos, cambia la URL por la del proyecto y luego entra a la carpeta usando `cd nombre-del-repo`).*


**IMPORTANTE: Las ramas (Branches):**

Yo me encargaré de gestionar las ramas (crearlas principalmente). Por favor **no subir nada directo a main.** Todos nuestros cambios irán directo a **dev**.

Una vez hayas clonado el repo, ejecuta esto para cambiar de rama:

```bash
git checkout dev
```

Si en el futuro necesitas cambiar a otra rama que hayamos creado, solo usa `git checkout nombre-de-la-rama`.

---

## 2. Nuestro Flujo de Trabajo Diario

Cada vez que vayas a trabajar en el código, sigue este orden **exacto**:

### Paso 1: Sincronizar (¡Siempre haz esto antes de empezar!)
Antes de escribir una sola línea de código, asegúrate de descargar los cambios que hicieron los demás para evitar conflictos.

```bash
git pull origin dev
```

### Paso 2: Realizar los cambios
Haz tus cambios, edita los archivos, y prueba que todo funcione bien en el robot o en la simulación.

### Paso 3: Revisar qué cambió
Para ver exactamente qué archivos modificaste, usa este comando (git mostrará los diferentes cambios):

```bash
git status
```

*Los archivos modificados aparecerán en color rojo o resaltados.*

### Paso 4: Preparar los cambios
Añade los archivos que quieres subir. Para añadir **todos** los cambios a la vez (lo más rápido y común):

```bash
git add .
```

*(Si vuelves a ejecutar `git status`, ahora los archivos estarán en verde, es como si ya hubieran sido registrados).*

### Paso 5: Empaquetar y etiquetar (El Commit)
Ahora "empaquetamos" esos cambios con un mensaje claro de lo que hicimos. **Recuerden ser descriptivos con los cambios que añadió cada quien. Podrían ser de ayuda en el futuro.**
`bash
git commit -m "[Descripcion]"
`

### Paso 6: Subir a GitHub
Finalmente, mandamos el paquete a la nube para que todo el equipo lo tenga disponible:

```bash
git push origin dev
```

---

## 3. Troubleshooting

* **"Merge Conflict" (Conflicto de fusión):** Si al hacer `pull` o `push` la terminal te lanza un error gigante con la palabra **CONFLICT**, ¡no entres en pánico! Solo significa que alguien más del equipo modificó exactamente la misma línea de código que tú. Avísanos por el grupo y lo resolvemos rápido.
* **Me pide contraseña al hacer push:** GitHub ya no acepta contraseñas normales en la terminal por seguridad, usa "Tokens". Si te pide contraseña, necesitas generar un "Personal Access Token" en tu cuenta de GitHub y pegarlo ahí.

### Pasos para generar un token:

1. Ve a la página de GitHub en tu navegador e inicia sesión.
2. Haz clic en tu foto de perfil y selecciona Settings.
3. En el menú de la izquierda, baja hasta el final y haz clic en Developer settings.
4. Haz clic en Personal access tokens y luego selecciona Tokens.
5. Haz clic en el botón Generate new token -> Generate new token (classic).
6. Ponle un nombre para que lo recuerdes (ej. "Terminal"), y en Expiration te recomiendo poner 90 días.
7. En la sección de Select scopes, marca la casilla que dice repo (esto te da permisos para subir código).
8. Baja al final y dale a Generate token.
9. **¡CÓPIALO INMEDIATAMENTE!** Te aparecerá un código largo. Guárdalo en un bloc de notas seguro o en tu gestor de contraseñas, porque **GitHub no te lo volverá a mostrar**.

La próxima vez que la terminal te pida contraseña al hacer git push, simplemente pega ese token largo (nota: al pegar en la terminal no se ven los caracteres por seguridad, pero sí se pega) y dale Enter.

---

## 4. Cómo fusionar ramas (Hacer un Merge)

Cuando terminamos una etapa del proyecto y todo funciona perfecto en `dev`, necesitaremos pasar todo ese código terminado a nuestra rama principal (`main`). Es decir hacer un *"merge"*.

Si necesitas hacer este proceso, sigue estos pasos con mucho cuidado. La regla de oro del Merge es: **Primero te paras en la rama que va a recibir los cambios, y desde ahí, "absorbes" a la otra.**

Ejemplo: Queremos pasar todo lo de `dev` hacia `main`.

### Paso 1: Cambiar a la rama receptora
Mueve tu terminal a la rama que va a recibir el código final (en este caso, `main`):

```bash
git checkout main
```

### Paso 2: Actualizar la rama receptora
Asegúrate de tener la versión más reciente de `main` en tu computadora antes de combinar nada:

```bash
git pull origin main
```

### Paso 3: ¡Hacer la fusión!
Ahora dile a Git que traiga y absorba todo el historial de la rama `dev` hacia donde estás parado ahorita (`main`):

```bash
git merge dev
```

*(Si todo sale bien, verás un mensaje diciendo "Fast-forward" o un resumen de los archivos que se añadieron).*

### Paso 4: Subir la fusión a la nube
El merge se hizo localmente en tu compu. Ahora hay que avisarle a GitHub que `main` ya tiene el nuevo código:

```bash
git push origin main
```

¡Y listo! Ahora `main` y `dev` están sincronizados y al día. Una vez terminado, recuerda regresar a `dev` (`git checkout dev`) para seguir programando.



**Con esta guía deberíamos de tener un buen workflow. Cualquier cosa lo checamos en el grupo**


# Docker en la Rubik Pi:

**🐳 Flujo de Trabajo: Docker + micro-ROS**

Para compilar y correr nuestro código sin problemas de dependencias, utilizamos un contenedor de Docker personalizado (`ros2_microros`). Este contenedor ya tiene todo preinstalado (ROS2 Humble + micro-ROS Agent).

Sigue este flujo de trabajo cada vez que vayas a probar el robot:

### 1. Verificación de Hardware (¡Muy Importante!)
Antes de lanzar Docker, debemos asegurarnos de que la placa host reconozca el microcontrolador físicamente.
Conecta el microcontrolador por USB y ejecuta en la terminal normal:

```bash
ls /dev/ttyUSB* /dev/ttyACM*
```

* **Si sale un error (`No such file`):** Hay un problema físico. Revisa el cable USB, cambia de puerto o desconecta/conecta de nuevo hasta que el sistema lo detecte. **No avances al paso 2 si esto falla.**
* **Si te devuelve una ruta (ej. `/dev/ttyUSB0`):** ¡Todo perfecto! Memoriza ese nombre.

### 2. Iniciar el Contenedor Principal (Terminal 1)
Una vez confirmada la conexión física, levanta la "burbuja" de ROS ejecutando nuestro alias principal:

```bash
iniciar_ros
```

*Nota: Este comando monta la carpeta actual dentro de Docker, mapea los puertos USB y crea el contenedor con el nombre `burbuja_ros`. Mientras esta terminal siga abierta, el contenedor existirá.*

### 3. Lanzar el Agente de micro-ROS
Dentro del contenedor (tu terminal dirá `root@...`), ejecuta el puente de comunicación usando el puerto que descubriste en el Paso 1:

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```

Verás que la terminal dice `Session established` cuando la conexión sea exitosa. ¡Déjala corriendo!

### 4. Abrir Terminales Adicionales (Terminal 2, 3, etc.)
Si necesitas correr más nodos, compilar código o ver tópicos (`ros2 topic list`), **NO uses `iniciar_ros` de nuevo** (eso gastaría el doble de RAM). 
Abre una nueva sesión SSH en tu computadora y usa nuestra "puerta trasera" para entrar al mismo contenedor que ya está corriendo:

```bash
entrar_ros
```

Puedes abrir tantas terminales como necesites usando este comando.

### 5. Salir y Limpiar
* **Para salir de una terminal secundaria (Terminal 2, 3...):** Simplemente escribe `exit`. Saldrás del contenedor, pero todo seguirá funcionando normalmente en el fondo.
* **Para apagar todo (Terminal 1):** Presiona `Ctrl + C` para detener el agente de micro-ROS y luego escribe `exit`. Esto destruirá el contenedor limpiamente y liberará la memoria.


*El monte Everest no tiene nada en contra de nosotros* 🗿🏔️