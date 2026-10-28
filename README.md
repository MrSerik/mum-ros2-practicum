# Практикум ROS 2: механика управляемых машин

Учебное рабочее пространство ROS 2 для практических работ курса «Механика управляемых машин»: **трёхзвенный манипулятор с сервоприводами** и **мобильный робот с дифференциальным приводом** в симуляторе Gazebo, планирование движений в **MoveIt 2** и навигация **Nav2**.

Платформа: **Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic**.

| Робот | Что умеет |
| --- | --- |
| `arm3` — манипулятор, 3 сервопривода | URDF по таблице Денавита–Хартенберга, ros2_control (позиция / момент), Gazebo, MoveIt 2 |
| `diffbot` — мобильный робот | Дифференциальный привод, 2D-лидар, RGB-D камера, полигон 8 × 6 м, SLAM и Nav2 |

## Подготовка

Установка ROS 2 Jazzy, Gazebo, Nav2, MoveIt 2 и всех нужных пакетов — один скрипт (≈ 3 ГБ, 15–30 минут):

```bash
curl -sLO https://raw.githubusercontent.com/MrSerik/mum-ros2-practicum/main/install_jazzy.sh
bash install_jazzy.sh
```

После установки откройте **новый** терминал и проверьте: `ros2 run turtlesim turtlesim_node` — должно появиться окно с черепахой.

> **Компьютерный класс.** Чтобы роботы соседей не принимали ваши команды, задайте свой номер домена (1…100):
> `echo "export ROS_DOMAIN_ID=<номер вашего ПК>" >> ~/.bashrc`

## Сборка рабочего пространства своими руками — 3 шага

| Шаг | Что создаём | Проверка в конце шага |
| --- | --- | --- |
| [**Шаг 1. Манипулятор arm3**](docs/step1_manipulator.md) | Рабочее пространство `~/mum_ws`, пакеты `arm3_description` и `arm3_moveit_config` | Рука двигается в Gazebo по траектории, MoveIt планирует движение |
| [**Шаг 2. Мобильный робот diffbot**](docs/step2_mobile_robot.md) | Пакет `diffbot_description`: модель, лидар, камера, мир Gazebo | Робот ездит с клавиатуры, по лидару строится карта |
| [**Шаг 3. Узлы практических работ**](docs/step3_lab_nodes.md) | Пакет `mum_labs`: кинематика, траектории, регуляторы, Nav2, зрение | Узлы рисуют графики, Nav2 ведёт робота к цели |

В каждом шаге объяснено, что делает каждый файл, и приведено его содержимое. Короткие и важные файлы набираются вручную, длинные служебные (настройки RViz, мир Gazebo) скачиваются командой `curl`.

**Сверка с эталоном.** Состояние репозитория после каждого шага отмечено тегами:

```bash
git clone https://github.com/MrSerik/mum-ros2-practicum.git ~/mum-ros2-reference
cd ~/mum-ros2-reference
git checkout step-1      # эталон после шага 1 (step-2, step-3 — после шагов 2 и 3)
git diff step-1 step-2 --stat   # какие файлы добавляет шаг 2
```

## Готовое рабочее пространство (если нужно сразу)

```bash
git clone https://github.com/MrSerik/mum-ros2-practicum.git ~/mum_ws
cd ~/mum_ws && ./install_jazzy.sh      # установит ROS 2 (если ещё нет) и соберёт пакеты
```

## Быстрая проверка

```bash
ros2 launch arm3_description display.launch.py              # модель манипулятора в RViz
ros2 launch arm3_description gazebo.launch.py rviz:=true    # манипулятор в Gazebo
ros2 launch arm3_moveit_config demo.launch.py               # MoveIt 2
ros2 launch diffbot_description gazebo.launch.py            # мобильный робот
ros2 launch mum_labs nav2_diffbot.launch.py slam:=True      # Nav2 + SLAM
```

## Практические работы

| № | Тема | Раздел лекций | Основная команда |
| --- | --- | --- | --- |
| 1 | Знакомство с ROS 2 и Gazebo | Введение | `ros2 launch diffbot_description gazebo.launch.py` |
| 2 | Прямая задача кинематики | 1.1–1.2 | `ros2 run mum_labs fk_node` |
| 3 | Обратная задача кинематики | 1.3 | `ros2 run mum_labs ik_node` |
| 4 | Матрица Якоби и сингулярности | 1.4 | `ros2 run mum_labs cartesian_line_node` |
| 5 | Планирование траектории полиномами | 2.3 | `ros2 run mum_labs trajectory_node` |
| 6 | Планирование движений в MoveIt 2 | 2.1–2.2 | `ros2 run mum_labs moveit_lab` |
| 7 | Регуляторы сочленений | 3.1–3.2 | `ros2 run mum_labs pid_node` |
| 8 | Мобильный робот: одометрия, выход в позу | 3 | `ros2 run mum_labs diff_odometry_node` |
| 9 | Навигация Nav2 | 2.2.1–2.2.2 | `ros2 launch mum_labs nav2_diffbot.launch.py` |
| 10 | Техническое зрение | 4 | `ros2 run mum_labs color_tracker` |

Задания, варианты и контрольные вопросы — в методических указаниях к практическим работам (выдаёт преподаватель). Графики и CSV, которые сохраняют узлы, попадают в `~/mum_results`.

Лекционный материал: Борисов О.И., Громов В.С., Пыркин А.А. *Методы управления робототехническими приложениями.* — СПб.: Университет ИТМО, 2016.

## Структура

```
mum_ws/
├── install_jazzy.sh            установка ROS 2 Jazzy и зависимостей
├── docs/                       пошаговые инструкции (шаги 1–3)
└── src/
    ├── arm3_description/       манипулятор: URDF, ros2_control, Gazebo
    ├── arm3_moveit_config/     MoveIt 2 для манипулятора
    ├── diffbot_description/    мобильный робот: URDF, датчики, мир, мост ROS–Gazebo
    └── mum_labs/               узлы практических работ (Python)
```

## Если что-то не работает

| Симптом | Решение |
| --- | --- |
| `install_jazzy.sh` останавливается с сообщением о недонастроенных пакетах | Сначала почините apt: `sudo dpkg --configure -a && sudo apt -f install`, затем запустите скрипт снова |
| `Package 'arm3_description' not found` | Не подключено рабочее пространство: `source ~/mum_ws/install/setup.bash` |
| Gazebo очень медленный, чёрный экран в виртуальной машине | Запускайте на реальном ПК; в ВМ — `export LIBGL_ALWAYS_SOFTWARE=1` |
| После прошлого запуска два робота, странные топики | `pkill -f "gz sim"; ros2 daemon stop` и запустить заново |
| Рука не двигается от команд | `ros2 control list_controllers` — нужен active `arm_controller` (режим `control_mode:=position`) |
| Правки в файлах не применяются | Сборка должна быть с `--symlink-install`; новый файл — снова `colcon build --symlink-install` |

Лицензия кода: Apache-2.0.
