
## Descripción

Este paquete permite que un robot con un gripper de succión pueda agarrar objetos. La razón de hacerlo es que el plugin existente era para gazebo classic y ya no funcionaba en las versiones más nuevas. Es útil cuando quieres simular manipuladores que usan ventosas de vacío sin complicarte demasiado con la dinámica de contacto. 

La idea es que cuando el vacío está activado y hay un objeto tocando el link de succión, el plugin lo 'agarra' aplicándole comandos de velocidad para que siga al gripper. Cuando el vacío se apaga, el objeto se libera y la física lo controla de nuevo.

## Estructura del paquete

El paquete está dividido en dos partes principales:
-  Librería C++ (VacuumGripper): el plugin de Gazebo.
-  Nodo Python (vacuum_bridge_node): un nodo de ROS 2 que actúa de puente, convirtiendo mensajes ROS 2 std_msgs/Bool en mensajes Gazebo Transport gz.msgs.Boolean. 

## El plugin C++ (VacuumGripper)

#### Configure()
Es el constructor funcional del plugin. Lee los parámetros del SDF, busca el link de succión en el modelo y se suscribe a los topics de Gazebo. Si algo falla (por ejemplo, no encuentra el link), el plugin se queda en estado configured = false y no hace nada más.

Parámetros que se pueden configurar desde el SDF:

| **Parámetro SDF** | **Tipo** | **Default**                 | **Descripción**                                                 |
| ----------------- | -------- | --------------------------- | --------------------------------------------------------------- |
| suction_link      | string   | vacuum_link_suction_gripper | Nombre del link del gripper                                     |
| contact_topic     | string   | /suction_contact            | Topic de contactos del sensor                                   |
| cmd_topic         | string   | /vacuum_on                  | Topic de comando on/off                                         |
| max_distance      | double   | 0.25                        | Distancia máxima de agarre (m)                                  |
| allowed_prefixes  | string   | (vacío = todos)             | Lista CSV de prefijos de modelo permitidos (ej: caja, box, etc) |
#### OnContacts() — callback del sensor
Se llama cada vez que el sensor de colisión detecta contactos. Guarda los candidatos (entidades en contacto + punto de contacto en coordenadas mundo)

#### PostUpdate() — detección de agarre
Aquí se decide si se debe iniciar un agarre. El proceso es:
- Si el vacío está apagado o ya está agarrando algo, no hace nada.
- Obtiene la posición mundial del link de succión.
- Para cada candidato de contacto, resuelve el modelo top-level, comprueba que no sea el propio robot, verifica que no sea estático y que su nombre cumpla los prefijos permitidos.
- De todos los candidatos válidos, elige el más cercano al suction_link cuya distancia sea menor que max_distance.
- Si encuentra uno, activa el estado holding = true y pone needsOffsetInit = true para que PreUpdate calcule el offset en el siguiente tick.

#### PreUpdate() — controlador PI
Es el núcleo del sistema de agarre. Corre antes de que el motor de física haga sus cálculos.
- Si el vacío se apaga, llama a Release() para soltar el objeto.
- Si es el primer tick tras el agarre (needsOffsetInit), calcula el offset relativo entre el suction_link y el objeto. Esto es lo que permite que el objeto 'siga' al gripper manteniendo su posición relativa.
- Calcula el error entre la posición objetivo (suction_pose * offset) y la posición actual del objeto.
- Aplica un controlador PI con anti-windup para generar comandos de velocidad lineal y angular.

Se usan velocidades y no poses ya que  el motor de física (Bullet/DART) sobreescribe el componente Pose en cada step. En cambio, los componentes LinearVelocityCmd y AngularVelocityCmd sí son leídos por el sistema Physics como entradas, así que son el mecanismo correcto.

Los parámetros del controlador son: kP = 1.5/dt y kI = 0.8/dt. El anti-windup satura el integrador a kIMaxLin = 2.0 m/s y kIMaxAng = 1.5 rad/s. Si se hacen movimientos más rápido puede que haya que subir los límites.

#### Release()
Limpia todo el estado del agarre: pone holding = false, borra la entidad retenida y resetea los integradores del PI. Lo más importante: elimina los componentes LinearVelocityCmd y AngularVelocityCmd del objeto soltado, para que el motor de física recupere el control completo.

## Compilación e instalación

```bash
cd ~/ros2_ws/src

# Colocar el paquete aquí

cd ~/ros2_ws

colcon build --packages-select vacuum_gripper_plugin

source install/setup.bash
```

## Cómo usarlo

Para añadir el plugin a un modelo, hay que incluir el bloque plugin dentro del tag model del SDF del robot.

```xml

<plugin filename="libVacuumGripper.so" name="vacuum_gripper_plugin::VacuumGripper">

	<suction_link>vacuum_link_suction_gripper</suction_link>
	
	<!-- robot_b es el prefijo de mi robot, de todas formas el nombre del topic es personalizable -->
	
	<contact_topic>/robot_b/suction_contact</contact_topic>
	
	<cmd_topic>/robot_b/vacuum_on</cmd_topic> 
	
	<max_distance>0.20</max_distance>
	
	<allowed_prefixes>caja_, objeto_</allowed_prefixes>

</plugin>
```

Además, el link especificado en suction_link debe tener un sensor de contacto configurado que publique en el topic indicado en contact_topic.

```xml
<gazebo reference="${prefix}link_suction_gripper">

	<sensor name="${prefix}suction_contact_sensor" type="contact">
	
	<always_on>true</always_on>
	
	<update_rate>50</update_rate>
	
	<contact>
	
	<collision>${prefix}link_suction_gripper_fixed_joint_lump__${prefix}body_collision_collision</collision>
	
	</contact>
	
	</sensor>

</gazebo>
```
Puntos importantes de esta configuración:
- El atributo reference debe coincidir exactamente con el nombre del link definido en suction_link del SDF del plugin (con el mismo prefix).
- El sensor publica en el topic definido en contact_topic del plugin. Por defecto /suction_contact, pero debe coincidir con lo que se configure en el SDF del plugin.
- El nombre de la collision dentro de <contact> lo genera el parser de URDF automáticamente al fusionar links con fixed joints. Si el modelo cambia, este nombre puede cambiar también hay que verificarlo con gz model --info o mirando el SDF generado.

## Limitaciones

- El agarre no es físicamente exacto: el controlador PI puede generar vibraciones si los parámetros kP/kI son muy altos o si la masa del objeto es muy grande.
- Solo puede agarrar un objeto a la vez. No hay soporte para agarrar varios simultáneamente.
- El topic del bridge está hardcodeado a /robot_b/vacuum_on_ros. Si se tienen varios robots, hay que modificar el nodo o parametrizarlo.
- Si el objeto es muy ligero o tiene física inestable, el controlador puede hacer que 'vibre' o se escape. Ajustar max_distance y los topes del integrador puede ayudar.

## Nodo bridge Python
Si se quiere utilizar el puente de gazebo a ros2 se debe lanzar este comando

```bash
ros2 run vacuum_gripper_plugin vacuum_bridge_node
```
.