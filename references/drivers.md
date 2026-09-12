# 硬件驱动层生成规范

## 目的和边界

硬件驱动层位于通信增强层、HAL 平台层与应用层之间。机器人系统中的常见硬件设备原则上抽象为两类：**传感器**和**执行器**。传感器向上提供实时数据及数据更新事件；执行器向上提供控制能力、运行状态和动作事件，默认主要覆盖各种电机及电机类部件。每个具体器件均封装成可独立创建、初始化和使用的硬件对象。

无法自然归入两类的器件，应先判断它对应用层的主要能力：以采集和上报数据为主的归入传感器，以接收目标并产生动作或输出为主的归入执行器。电机模组等将传感、状态反馈和驱动能力封装在同一物理器件、共用同一通信协议与生命周期的一体化器件，应使用同一个驱动对象和同一组驱动文件，同时定义反馈数据、控制接口和事件回调，不得为了套用分类而强行拆成传感器对象与执行器对象。只有各部分能够独立初始化、独立使用，或具有明确不同的资源与生命周期时，才拆分为相互关联的对象；无论采用哪种组织方式，都不得把应用业务引入驱动层。

硬件驱动层负责：

- 器件初始化及其所需的 HAL 或通信增强层资源绑定；
- 器件协议的封装与解析；
- 将原始数据解析为具有明确含义、单位和方向约定的器件状态量；驱动层通常保留脉冲、计数值等器件原生单位，不强制换算为米、毫米、度或弧度等与具体机构相关的物理单位；
- 单个器件内部必要的控制、保护、错误状态和事件输出；
- 同型号多个硬件实例之间相互独立的运行数据管理。

硬件驱动层不得：

- 实现机器人工作模式、应用流程、任务编排或多个器件之间的业务决策；
- 绕过通信增强层重复实现 UART、CAN 等公共发送缓冲和接收分发机制；
- 修改 STM32 HAL 官方源码；
- 默认引入 LL 库或直接操作外设寄存器；
- 通过隐含的全局变量、调用顺序或优先级关系与其他驱动耦合。

## 上位规则

生成或修改硬件驱动前，必须先读取并遵守：

- `references/project-rules.md`：全工程共同遵守的头文件封装、延时、优先级、执行上下文、HAL 和资源所有权规则；
- `references/architecture-contract.md`：硬件驱动层与平台层、通信增强层及应用层的依赖边界；
- `references/comms.md`：驱动使用 UART、CAN、SPI、I2C 等通信资源时的公共增强接口和接收绑定方式；
- `references/standard.md`：项目现有的命名、结构体和源文件组织约定。

开发具体驱动时，用户必须提供对应硬件的开发手册、通信协议或数据手册，以及当前项目中的资源连接信息。实现前必须从这些资料中确认寄存器或报文定义、数据类型与字节序、量程与单位、通信时序、初始化流程、错误码和安全限制。资料没有说明的协议字段、控制命令、资源映射和安全参数不得猜测；缺少完成实现所必需的资料时，应明确指出缺项并向用户索取。

本文件只能在上述全工程规则范围内补充硬件驱动层的专用约束。

## 驱动对象与封装

每个可独立使用的器件实例应由一个公开对象结构体表示。头文件是驱动开发者与驱动使用者之间的边界：使用者只接触器件状态、允许修改的配置、控制接口和事件出口；开发者可以在不破坏公开接口的前提下修改协议过程、缓存、计时量和内部控制逻辑。应用层只阅读头文件即可正确使用驱动，不应依赖源文件实现细节。

### 对象分类与公共结构

驱动对象可按需要包含以下公共区域：

1. **ReadOnly 实时数据或状态区**：由驱动内部更新，使用者只读；必须注明单位、有效范围、无效值和方向约定。
2. **ReadWrite 配置区**：仅在器件确有需要时提供，必须说明有效范围和允许修改的时机。
3. **Private 私有数据入口**：公开结构体只保存不透明私有结构体指针 `priVari`。头文件只前置声明私有类型，不公开其成员定义。
4. **父级对象入口**：`fatherArgs` 是由上层装配时写入的父对象上下文指针，使子对象触发事件时，上层回调能够定位并操作该子对象所属的组合单元。它不表示内存所有权，子驱动不得释放、解释或直接操作它指向的对象。
5. **事件回调区**：传感器至少关注实时数据更新事件；执行器可按能力提供状态更新、动作完成、到位、回零和故障等事件。回调的第一个参数固定为触发事件的当前硬件对象，第二个参数固定传入该对象的 `fatherArgs`，从而同时提供“哪个子对象发生事件”和“它属于哪个父对象”两部分上下文。

### `fatherArgs` 与父级联动

`fatherArgs` 主要用于子单元发生事件后向上层反向定位父单元，并由父层执行组合控制。例如一根手指由三个电机协同驱动，三个电机的 `fatherArgs` 都可以指向同一个 `Finger` 对象。当任意电机报告过载时，电机驱动触发错误回调；父层实现的回调通过 `fatherArgs` 找到整根手指，将手指状态机切换到停止伸出或安全回退状态，再由手指的周期处理函数协调停止三个电机。

反向操作的边界必须保持清晰：电机驱动只负责报告“当前电机发生错误”并原样传递 `fatherArgs`，不得包含 `Finger` 的定义、把 `fatherArgs` 强制转换成 `Finger *`，也不得自行决定另外两个电机如何动作。类型转换和整根手指的联动策略属于创建、组合这些电机的父层。这样同一电机驱动既可以用于手指，也可以用于机械臂、夹爪或独立测试设备。

```c
typedef enum {
    FINGER_STATE_IDLE = 0,
    FINGER_STATE_EXTENDING,
    FINGER_STATE_STOP_EXTENDING
} Finger_State;

typedef struct {
    Actuator *motor[3];
    Finger_State state;
    Finger_State requestedState;
} Finger;

/* 该函数属于 Finger 控制层，不属于电机驱动层。 */
static void Finger_MotorErrorCallBack(
    Actuator *errorMotor,
    void *fatherArgs)
{
    Finger *finger = (Finger *)fatherArgs;

    if ((finger == NULL) || (errorMotor == NULL)) {
        return;
    }

    if (Actuator_IsOverload(errorMotor)) {
        /* 回调只提交父状态机请求，避免在接收中断中连续操作三个电机。 */
        finger->requestedState = FINGER_STATE_STOP_EXTENDING;
    }
}

static void Finger_BindMotors(Finger *finger)
{
    uint32_t index;

    for (index = 0U; index < 3U; ++index) {
        finger->motor[index]->fatherArgs = finger;
        finger->motor[index]->errorCallBack = Finger_MotorErrorCallBack;
    }
}

void Finger_Process(Finger *finger)
{
    if ((finger->requestedState == FINGER_STATE_STOP_EXTENDING) &&
        (finger->state != FINGER_STATE_STOP_EXTENDING)) {
        /* 在父层周期上下文中协调所有子执行器。 */
        Actuator_Stop(finger->motor[0]);
        Actuator_Stop(finger->motor[1]);
        Actuator_Stop(finger->motor[2]);
        finger->state = FINGER_STATE_STOP_EXTENDING;
    }
}
```

父对象的生命周期必须覆盖所有绑定子对象可能触发回调的时间。绑定应在启用硬件和注册接收入口之前完成；解绑或销毁父对象前，应先停止事件源并清空子对象的回调和 `fatherArgs`。允许 `fatherArgs == NULL`；只要回调已绑定，驱动仍按统一签名调用，回调必须检查该空指针。若回调运行在 ISR 或通信接收上下文，应像上例一样只投递父状态机请求或事件，不得在回调内执行耗时、阻塞或多器件连续控制。

### 传感器对象

传感器公开结构体的主要内容是实时数据和实时数据更新后的软中断函数。这里的“软中断”指驱动在一组新数据解析并发布后调用的事件回调，不等同于 MCU 的硬件中断或软件中断指令。回调实际运行在 ISR、定时器或任务上下文，取决于数据更新路径，驱动必须明确说明。

大多数传感器应通过器件主动上报、周期查询、ADC/DMA 周期采样或其他方式保持稳定的基础更新频率。每次有效数据发布后的回调可以作为应用层控制算法的基础节拍，相当于一个由传感器数据驱动的软定时器。驱动必须说明其标称周期、允许抖动、超时判断和丢帧行为；只有更新频率确实稳定且数据有效时，应用层才能依赖该回调形成控制节拍。发生丢帧、校验失败或通信超时时不得照常触发数据更新回调，安全相关控制还必须具有独立的超时保护。

传感器的数据接收、解析和发布分为三个阶段：

1. **识别**：检查通信端口、CAN ID、设备地址、帧头、功能码、长度等数据识别标头，判断数据是否属于当前类型和当前实例。
2. **解析**：在局部变量或私有暂存区中完成字节序调整、校验、协议字段解析、滤波和派生量计算。公开实时数据不得作为解析过程的中间变量。
3. **发布**：全部字段解析成功后，将结果写入公开实时数据区，随后触发一次数据更新回调。

公开实时数据允许逐字段更新，不额外使用短临界区、序列计数器或双缓冲来保证多字段一致快照。应用层并发读取时可能短暂读到分别来自相邻两次更新的数据；对于更新频率较高的传感器，这种差异通常可以接受。解析过程仍应先在局部变量或私有暂存区中完成，避免校验失败或数据包不完整时污染公开数据。

当通信帧到达频率高于系统实际需要时，可以相对基础接收频率进行整数分频，只在每 N 个有效帧中解析并发布一次数据，以降低 CPU、滤波和应用回调开销。即使跳过本帧的完整解析，接收入口仍须完成归属识别和必要的长度/边界检查，并向通信增强层正确报告该帧是否已被本驱动认领。分频值、实际发布周期和计数器初值必须明确，不能造成首帧时机不确定或速度、积分等时间相关计算使用错误周期。

每类传感器通常提供一个由数量宏决定大小的硬件对象数组，用于保存当前项目中的全部同型号实例。数组容量和实例数量必须由项目配置确定，初始化时检查索引、设备 ID 和重复注册；数组中的每个对象及其私有状态相互独立。若通信增强层采用逐对象注册，也可以保存对象指针数组，但不得用数组中的共享临时解析变量破坏多实例隔离。

标准定义格式如下。实际生成时把 `Sensor` 和数据字段替换成具体器件名称与实际数据：

```c
/* 开发者使用的内部变量由私有结构体保护。
 * 这里只做前置声明，使使用者无法访问或修改其成员。 */
typedef struct _Sensor_Private Sensor_Private;

/* 提前声明公开对象，供回调函数类型使用。 */
typedef struct _Sensor Sensor;

typedef void (*Sensor_CallBack)(
    Sensor *sensor,
    void *fatherArgs);

typedef struct {
    int32_t value;   /* 示例单位：器件协议定义的原生计数 */
    uint16_t status; /* 示例：器件状态字 */
} Sensor_Data;

struct _Sensor {
    /* RealTimeData / ReadOnly：一组实时数据，由驱动更新，使用者只读。 */
    Sensor_Data data;

    /* Private：只能由 Sensor.c 内部访问成员。 */
    Sensor_Private *priVari;

    /* 用于应用层多级对象组合。 */
    void *fatherArgs;

    /* 一组实时数据全部更新后触发一次。 */
    Sensor_CallBack refreshDataCallBack;
};
```

推荐的数据包解析和发布骨架如下：

```c
#define SENSOR_NUM 4U
#define SENSOR_PARSE_DIV 2U
#define SENSOR_FRAME_HEAD 0xA5U
#define SENSOR_FRAME_SIZE 6U

static Sensor sensorList[SENSOR_NUM];

typedef struct {
    int16_t value;
    uint16_t status;
} Sensor_FrameFields;

typedef union {
    Sensor_FrameFields fields;
    uint8_t bytes[sizeof(Sensor_FrameFields)];
} Sensor_Frame;

static bool Sensor_RxCallback(const uint8_t *data, uint16_t size, void *object)
{
    Sensor *sensor = object;
    Sensor_Private *privateData = sensor->priVari;

    /* 数据识别标头用于配合 comms 层完成接收回调分发。 */
    if ((size != SENSOR_FRAME_SIZE) ||
        (data[0] != SENSOR_FRAME_HEAD) ||
        (data[1] != privateData->deviceId)) {
        return false; /* 不是当前实例的数据。 */
    }

    if (++privateData->parseCount < SENSOR_PARSE_DIV) {
        return true;  /* 已认领该帧，但本周期不做完整解析。 */
    }
    privateData->parseCount = 0U;

    Sensor_Frame frame;
    Sensor_Data nextData;

    /* 按开发手册规定的字节序填充，不直接假定线上字节序。 */
    frame.bytes[0] = data[3];
    frame.bytes[1] = data[2];
    frame.bytes[2] = data[5];
    frame.bytes[3] = data[4];

    /* 只操作临时结果，不在解析过程中改写 sensor->data。 */
    nextData.value = frame.fields.value;
    nextData.status = frame.fields.status;

    Sensor_Publish(sensor, &nextData); /* 将解析结果写入公开数据区。 */
    if (sensor->refreshDataCallBack != NULL) {
        sensor->refreshDataCallBack(sensor, sensor->fatherArgs);
    }
    return true;
}
```

对于电机模组等传感与驱动一体化器件，上述反馈接收、解析和发布逻辑应与控制逻辑写在同一个具体器件驱动中，公开对象也同时包含反馈状态和控制接口。其 `Actuator_Process` 应由主循环或统一周期任务调用，并可在实现文件中紧接反馈解析函数组织，使数据更新、故障判断和控制状态机的关系清晰。具体流程仍由硬件手册唯一确定：主动上报型器件在接收入口解析并发布状态；一问一答型器件在自身的非阻塞周期逻辑中发起查询并异步处理应答。接收中断只完成必要的数据搬运、帧识别和事件投递，不得直接执行轨迹规划、连续通信或应用回调。

`union` 适合把固定长度字节数组与协议字段视图放在一起，但必须使用 `uint8_t`、`int16_t`、`uint32_t` 等定宽类型，并依据开发手册显式处理字节序、符号位和缩放关系。若结构体存在填充、未对齐访问或编译器布局差异，不得直接把接收缓冲区强制转换为结构体指针；应逐字节填充、使用 `memcpy` 后转换，或逐字段解析，并通过编译期长度检查确认布局。

传感器应在本次解析结果全部写入公开数据区后触发一次 `refreshDataCallBack`。该要求用于明确回调时机，不承诺应用层并发读取多个字段时获得原子一致快照。没有新数据时不得伪造刷新事件。

### 执行器对象

执行器默认按电机或电机类部件设计。对于带编码器、电流检测、温度检测或其他反馈的一体化电机模组，这些传感数据属于同一个执行器对象，不再另外建立传感器对象。公开对象通常包含位置、速度、电流、使能状态、运行状态和错误状态等只读数据，并通过公开函数提供使能、停止、设定目标、连续轨迹、固定目标速度运行、实时目标跟踪、回零和清除错误等器件能力。会立即触发硬件动作或需要范围校验的目标值不得通过直接改写公开字段下发。

除使能、急停等简单且必须立即生效的命令外，控制接口只校验并写入私有目标参数和状态机请求，不得在接口内阻塞等待或连续下发硬件命令。实际命令由周期执行函数中的有限状态机统一发出，使应用层的突发调用先变成稳定、可仲裁的状态请求。若接口调用与周期执行不在同一上下文，目标参数和状态请求的提交必须使用工程统一的临界区或消息机制保证一致性。

连续控制和状态更新必须直接按照具体器件的硬件手册实现。器件主动周期上报状态时，在接收解析路径发布反馈；器件需要查询才应答时，由主循环按手册规定的周期发起查询。每个具体驱动只包含该器件实际需要的数据收发流程。

执行器可以按实际需要提供无限实时跟踪模式。该模式的启动接口接收一个目标变量指针并保存指针本身，而不是只复制启动时的值；周期执行函数每次运行都重新读取指针指向的最新目标，完成范围校验后更新本周期的规划位置或规划速度。目标变量通常可以直接指向传感器或一体化电机模组的公开实时数据，使执行器持续跟随数据刷新。固定目标的 `POSITION`、`VELOCITY` 模式与指针目标持续变化的 `TRACK_POSITION`、`TRACK_VELOCITY` 模式必须明确区分。

```c
typedef struct _Actuator_Private Actuator_Private;
typedef struct _Actuator Actuator;

typedef enum {
    ACTUATOR_STATE_UNINITIALIZED = 0,
    ACTUATOR_STATE_DISABLED,
    ACTUATOR_STATE_IDLE,
    ACTUATOR_STATE_POSITION,
    ACTUATOR_STATE_VELOCITY,
    ACTUATOR_STATE_CURRENT,
    ACTUATOR_STATE_TRAJECTORY,
    ACTUATOR_STATE_TRACK_POSITION,
    ACTUATOR_STATE_TRACK_VELOCITY,
    ACTUATOR_STATE_HOMING,
    ACTUATOR_STATE_STOPPING,
    ACTUATOR_STATE_ERROR
} Actuator_State;

typedef enum {
    ACTUATOR_TRACK_POSITION = 0,
    ACTUATOR_TRACK_VELOCITY
} Actuator_TrackingType;

typedef void (*Actuator_CallBack)(
    Actuator *actuator,
    void *fatherArgs);

struct _Actuator {
    /* ReadOnly：由驱动更新，使用者只读。 */
    int32_t nowPosi; /* 单位：pulse */
    int32_t nowVelo; /* 单位：pulse/s */
    int32_t nowCurrent; /* 单位：mA；器件不支持时删除。 */
    Actuator_State state;
    bool enabled;
    uint32_t errorCode;

    Actuator_Private *priVari;
    void *fatherArgs;

    Actuator_CallBack refreshDataCallBack;
    Actuator_CallBack actionFinishedCallBack;
    Actuator_CallBack errorCallBack;
};

bool Actuator_Init(Actuator *actuator);
bool Actuator_Enable(Actuator *actuator, bool enable);
void Actuator_Stop(Actuator *actuator); /* 普通停止；急停接口按硬件另行提供。 */
bool Actuator_SetPosition(Actuator *actuator, int32_t position);
bool Actuator_SetVelocity(Actuator *actuator, int32_t velocity);
bool Actuator_SetCurrent(Actuator *actuator, int32_t current);
bool Actuator_StartTrajectory(Actuator *actuator,
                              const int32_t *positionList,
                              uint32_t pointCount);
bool Actuator_StartTracking(Actuator *actuator,
                            const volatile int32_t *targetSource,
                            Actuator_TrackingType trackingType);
bool Actuator_Home(Actuator *actuator);
bool Actuator_ClearError(Actuator *actuator);

/* 由主循环或统一调度任务周期调用，不得在其中阻塞。 */
void Actuator_Process(Actuator *actuator);
```

具体执行器只保留器件真实支持且应用层需要的状态、控制函数和回调，不为凑齐模板制造无意义字段。非电机执行器也遵循相同边界，按其实际能力替换位置、速度和电流等字段。初始化函数使用硬件手册和项目配置中已经确定的具名硬件资源、设备地址及通信流程完成初始化。初始化完成后对象进入明确的 `DISABLED` 或安全状态；同一实例重复初始化不得重复占用资源。

对应的私有结构体完整定义只能放在相应的 `.c` 文件中：

```c
struct _Actuator_Private {
    Actuator_State requestedState;

    int32_t targetPosi;
    int32_t targetVelo;
    int32_t targetCurrent;
    const int32_t *trajectory;
    uint32_t trajectoryCount;
    uint32_t trajectoryIndex;
    const volatile int32_t *trackingSource;

    bool initialized;
    bool finishEventSent;
    bool errorEventSent;

    /* 仅加入该器件协议真实需要的通信缓存、计时量和硬件状态。 */
};

bool Actuator_Init(Actuator *actuator)
{
    Actuator_Private *privateData;

    if (actuator == NULL) {
        return false;
    }

    /* 私有槽由驱动内部的静态实例池管理，不把分配责任交给使用者。 */
    if ((actuator->priVari != NULL) && actuator->priVari->initialized) {
        return true; /* 已初始化实例不得再次占用或重配硬件资源。 */
    }
    privateData = Actuator_ClaimPrivateSlot(actuator);
    if (privateData == NULL) {
        return false;
    }

    privateData->requestedState = ACTUATOR_STATE_DISABLED;
    privateData->trajectory = NULL;
    privateData->trajectoryCount = 0U;
    privateData->trajectoryIndex = 0U;
    privateData->trackingSource = NULL;
    privateData->finishEventSent = true;
    privateData->errorEventSent = false;
    privateData->initialized = false;
    actuator->priVari = privateData;

    /* 直接初始化该型号已经确定的具名资源、地址和通信路径。 */
    if (!Actuator_HwInit(actuator)) {
        actuator->priVari = NULL;
        Actuator_ReleasePrivateSlot(privateData);
        return false;
    }

    actuator->nowPosi = 0;
    actuator->nowVelo = 0;
    actuator->nowCurrent = 0;
    actuator->errorCode = 0U;
    actuator->enabled = false;
    actuator->state = ACTUATOR_STATE_DISABLED;
    privateData->initialized = true;
    return true;
}

/* 公开操作接口的典型实现：只提交经过校验的目标和状态请求。 */
bool Actuator_SetPosition(Actuator *actuator, int32_t position)
{
    Actuator_Private *privateData;

    if ((actuator == NULL) || (actuator->priVari == NULL)) {
        return false;
    }
    privateData = actuator->priVari;
    if ((!privateData->initialized) || (!actuator->enabled) ||
        (!Actuator_IsPositionValid(position))) {
        return false;
    }

    privateData->targetPosi = position;
    privateData->trackingSource = NULL; /* 切换模式时解除旧跟踪源。 */
    privateData->finishEventSent = false;
    privateData->requestedState = ACTUATOR_STATE_POSITION;
    return true;
}

bool Actuator_StartTracking(
    Actuator *actuator,
    const volatile int32_t *targetSource,
    Actuator_TrackingType trackingType)
{
    Actuator_Private *privateData;

    if ((actuator == NULL) || (actuator->priVari == NULL) ||
        (targetSource == NULL) ||
        ((trackingType != ACTUATOR_TRACK_POSITION) &&
         (trackingType != ACTUATOR_TRACK_VELOCITY))) {
        return false;
    }
    privateData = actuator->priVari;
    if ((!privateData->initialized) || (!actuator->enabled)) {
        return false;
    }

    /* 保存数据源地址；实际目标值由 Actuator_Process 每周期重新读取。 */
    privateData->trackingSource = targetSource;
    privateData->finishEventSent = true; /* 无限模式不产生自然完成事件。 */
    privateData->requestedState =
        (trackingType == ACTUATOR_TRACK_POSITION)
            ? ACTUATOR_STATE_TRACK_POSITION
            : ACTUATOR_STATE_TRACK_VELOCITY;
    return true;
}

static void Actuator_FinishOnce(Actuator *actuator)
{
    Actuator_Private *privateData = actuator->priVari;

    actuator->state = ACTUATOR_STATE_IDLE;
    privateData->requestedState = ACTUATOR_STATE_IDLE;
    if (!privateData->finishEventSent) {
        privateData->finishEventSent = true;
        if (actuator->actionFinishedCallBack != NULL) {
            actuator->actionFinishedCallBack(actuator, actuator->fatherArgs);
        }
    }
}

void Actuator_Process(Actuator *actuator)
{
    Actuator_Private *privateData;

    if ((actuator == NULL) || (actuator->priVari == NULL)) {
        return;
    }
    privateData = actuator->priVari;
    if (!privateData->initialized) {
        return;
    }

    /* 使用已发布的最新反馈检查故障；所有函数都必须快速返回。 */
    if (Actuator_HardwareHasError(actuator)) {
        Actuator_EnterErrorOnce(actuator);
        return;
    }

    if (actuator->state != privateData->requestedState) {
        actuator->state = privateData->requestedState;
    }

    switch (actuator->state) {
    case ACTUATOR_STATE_POSITION:
        Actuator_HwSetPosition(actuator, privateData->targetPosi);
        if (Actuator_PositionReached(actuator, privateData->targetPosi)) {
            Actuator_FinishOnce(actuator);
        }
        break;

    case ACTUATOR_STATE_VELOCITY:
        Actuator_HwSetVelocity(actuator, privateData->targetVelo);
        break; /* 持续跟踪，直到收到新的状态请求。 */

    case ACTUATOR_STATE_CURRENT:
        Actuator_HwSetCurrent(actuator, privateData->targetCurrent);
        break; /* 持续跟踪，直到收到新的状态请求。 */

    case ACTUATOR_STATE_TRAJECTORY:
        if (privateData->trajectoryIndex < privateData->trajectoryCount) {
            Actuator_HwSetPosition(
                actuator,
                privateData->trajectory[privateData->trajectoryIndex++]);
        } else if (Actuator_PositionReached(actuator, privateData->targetPosi)) {
            Actuator_FinishOnce(actuator);
        }
        break;

    case ACTUATOR_STATE_TRACK_POSITION: {
        int32_t trackingTarget;

        if (privateData->trackingSource == NULL) {
            Actuator_RequestSafeStop(actuator);
            break;
        }
        trackingTarget = *privateData->trackingSource;

        if (!Actuator_IsPositionValid(trackingTarget)) {
            Actuator_RequestSafeStop(actuator);
            break;
        }
        Actuator_HwSetPosition(actuator, trackingTarget);
        break; /* 不自动完成，直到收到停止或其他状态请求。 */
    }

    case ACTUATOR_STATE_TRACK_VELOCITY: {
        int32_t trackingTarget;

        if (privateData->trackingSource == NULL) {
            Actuator_RequestSafeStop(actuator);
            break;
        }
        trackingTarget = *privateData->trackingSource;

        if (!Actuator_IsVelocityValid(trackingTarget)) {
            Actuator_RequestSafeStop(actuator);
            break;
        }
        Actuator_HwSetVelocity(actuator, trackingTarget);
        break; /* 不自动完成，直到收到停止或其他状态请求。 */
    }

    case ACTUATOR_STATE_HOMING:
        Actuator_HwHomeStep(actuator); /* 只推进一个控制周期，不循环等待。 */
        if (Actuator_HomeReached(actuator)) {
            Actuator_FinishOnce(actuator);
        }
        break;

    case ACTUATOR_STATE_STOPPING:
        Actuator_HwStop(actuator);
        if (Actuator_Stopped(actuator)) {
            Actuator_FinishOnce(actuator);
        }
        break;

    default:
        break;
    }
}
```

上例中的 `Actuator_HwSetPosition`、`Actuator_HwSetVelocity` 和 `Actuator_HwSetCurrent` 是器件真正支持的底层控制指令；连续轨迹、固定速度运行、实时目标跟踪和回零由状态机在这些原语之上逐周期推进。轨迹缓冲区的所有权和有效期必须在具体驱动接口中说明，资源受限或跨上下文时应复制到驱动自有缓冲区。

实时跟踪源指针在退出跟踪模式前必须始终有效并指向地址稳定的对象，不得指向已经离开作用域的局部变量、可被释放的临时内存或会被重新分配的缓存。停止、禁用、故障或切换到其他控制模式时，应退出跟踪状态并清空私有源指针。`volatile` 只保证周期函数重新读取目标值，不提供原子性或跨上下文同步；若目标由 ISR、DMA 或其他任务更新，且其宽度超过处理器可原子访问范围，具体驱动必须采用工程统一的临界区、序列计数器、双缓冲或快照接口取得一致数据。还必须结合数据有效标志或最后刷新时间检测源数据超时，超时后进入硬件规定的安全停止状态，不得无限沿用失效目标。

禁止用延时、忙等或在一次调用中循环发送完整轨迹。到位、轨迹终点、回零完成或受控停止完成时，先更新公开状态，再对该动作触发一次 `actionFinishedCallBack`；进入新动作时清除完成事件锁存。故障回调同样按一次故障沿触发，故障未清除时不得每周期重复调用。

### 封装约束

- 使用者不得解引用、替换或释放 `priVari`；它的创建、初始化、生命周期和释放全部由驱动实现管理。
- C 语言的不透明类型可以阻止使用者访问私有结构体成员，但不能阻止其改写指针本身，因此“不得修改 `priVari`”同时属于接口契约和代码审查规则。需要更强保护时，应改用完全不透明的对象句柄。
- ReadOnly 字段是面向使用者的只读契约，驱动仍可在内部更新。需要从语言层面强制保护的数据应移入私有结构体，并提供查询函数。
- ReadWrite 字段不得包含必须经过校验或会立即触发硬件动作的数据；此类修改必须通过公开设置函数完成。
- 回调函数是驱动向应用层输出事件的接口，不得在驱动内部反向嵌入父对象类型或应用业务。父层提供的回调可以通过 `fatherArgs` 操作父状态机，但必须遵守回调实际所在的 ISR、软件定时器或任务上下文限制；只有实际运行在 ISR 中的回调才使用 `_ISR_` 命名。
- 触发回调时统一使用 `callback(object, object->fatherArgs)`：`object` 标识发生事件的子对象，`fatherArgs` 定位所属父对象。驱动只原样传递 `fatherArgs`，不解释、不拥有且不释放它。回调为空时安全跳过，不得要求使用者必须绑定。
- 驱动应提供充分的初始化、控制、查询、配置以及必要的回调绑定和解绑接口，不得迫使使用者访问私有数据、源文件静态变量或直接调用底层 HAL。
- 同一型号存在多个器件时，每个实例必须拥有独立的公开对象和私有状态。除只读常量和明确设计的共享通信资源外，不得通过未归属的全局变量共享运行状态。

## 底层资源使用

- GPIO、ADC、PWM、定时器等资源默认通过 CubeMX 生成的 HAL 句柄和 HAL 公共 API 使用。
- UART、CAN 等共享通信资源优先通过 `FOS_Xxx` 通信增强层接口发送，并使用通信接收函数绑定机制接收属于本器件的数据。
- 驱动负责识别和解释器件协议，但通信增强层只负责传输和分发，不得把器件协议下沉到通信增强层。
- 一个驱动不得擅自重新初始化由平台层或通信增强层统一管理的共享外设。
- 外设句柄、设备地址、通道、GPIO 和 DMA 等资源映射必须来自项目配置或用户提供的硬件资料，不得猜测。

## 初始化

每类驱动应提供公开初始化函数。初始化入口允许被不同应用装配位置重复调用，但同一实例的资源只能完成一次有效初始化。

初始化至少负责：

- 校验对象、硬件句柄、设备地址和必要配置；
- 调用所依赖通信增强层的一次性初始化入口；
- 创建或绑定该实例的私有状态；
- 注册接收处理函数和必要的器件事件；
- 对执行器设置安全初始输出，并为传感器和执行器设置明确的初始状态；
- 返回可诊断的成功或错误结果。

后续重复调用不得重复分配私有状态、重复注册接收函数或重复创建任务、定时器和同步对象。初始化失败时不得留下看似可用的半初始化实例。

## 控制、状态与事件

- 控制接口表达器件能力，例如启动、停止、设定目标、清除错误和回零，不表达机器人业务动作。
- 所有状态量和控制量必须标明单位、有效范围和方向约定。驱动层通常使用 pulse、pulse/s、编码器计数或器件协议定义的原生单位；不得强制换算为米、毫米、度或弧度。依赖减速比、丝杆导程、连杆尺寸等特定机械结构的换算应由应用层或专门的机构抽象层完成。
- 状态更新必须明确来源、更新时机和调用上下文；由 ISR/HAL 回调触发时，只完成短操作或通知任务。
- 禁止使用 `HAL_Delay()`、忙等待或依赖任务优先级完成控制时序。任务上下文确需等待时，遵守 `project-rules.md` 的 `osDelay()` 规则。
- 事件回调应传入当前硬件对象，并允许通过 `fatherArgs` 找到所属上层对象；必须说明回调运行在 ISR、定时器还是任务上下文。
- 应用层能够绑定和解绑需要的事件，不得要求直接修改驱动私有状态实现事件控制。

## 参考与复用

开始实现前，在 `examples/` 中选择使用相同器件、相似控制对象或相同通信接口的代表性驱动，只读取相关头文件、源文件及其实际集成入口。

`examples/Cather-integer/Core/Inc/YS_Moto.h`、`Src/YS_Moto.c` 与对应的 `M2006.h`、`M2006.c` 是理解一体化电机驱动中“反馈数据 + 执行控制”共同封装方式的参考实例。其中可重点参考稳定更新节拍、刷新回调、`union` 数据视图、宏定义实例数量、接收分频以及 UART 帧头或 CAN ID 识别；其中与具体机械结构绑定的单位换算和控制参数不能直接作为新驱动的通用实现。

可以复用已经验证的对象封装、协议处理、状态更新和控制算法，但 MCU 型号、引脚、外设实例、设备数量、参数、限位和安全阈值必须根据当前项目重新确认。示例中的临时代码、调试入口、工程特例和已知缺陷不得直接升级为通用规则。

## 生成与检查

生成或修改一个硬件驱动时，至少检查：

- 是否已取得并阅读对应硬件的开发手册、通信协议或数据手册，必要资料是否完整；
- 器件是否已按主要能力正确归入传感器或执行器；传感与驱动一体化的器件是否保持在同一驱动对象中，只有能够独立初始化、独立使用或生命周期明确不同时才拆分；
- 驱动边界是否只覆盖单个器件或一类紧密相关的硬件能力；
- 传感器是否具有稳定且有文档说明的基础更新频率、超时和丢帧行为；
- 数据识别标头能否让 comms 层正确区分设备类型和具体实例；
- 数据是否先在局部变量或私有暂存区完成解析，再发布到公开实时数据区；
- 使用 `union` 时，定宽类型、字节序、符号、对齐和结构体长度是否符合开发手册及目标编译器；
- 传感器是否在一组数据全部发布后只触发一次刷新回调，应用层能否安全地用该回调作为控制节拍；
- 降频解析是否仍会正确认领数据帧，实际发布周期及时间相关计算是否同步调整；
- 宏定义数量与对象数组容量是否一致，初始化是否检查越界、重复 ID 和多实例隔离；
- 执行器的公开接口是否表达器件能力，且需要校验或会触发动作的控制是否通过函数下发；
- 实时跟踪模式是否每周期重新读取目标源，按跟踪类型校验最新位置或速度，并明确源指针生命周期、并发一致性、数据超时、退出条件及安全停止行为；
- 头文件是否充分服务使用者，同时隐藏开发者私有数据；
- 多实例是否拥有独立状态并能共享通信资源而互不干扰；
- HAL 与通信增强层接口是否使用正确，是否重复实现公共通信功能；
- 初始化是否幂等，失败和重复调用是否安全；
- 控制接口是否避免阻塞、隐式优先级和未声明的调用顺序；
- 状态、物理单位、错误和回调上下文是否明确；
- 示例复用内容是否已经按当前硬件资料完成适配；
- CubeMX 再生成代码后，驱动及其集成入口是否仍然保留。
