# Coding Guideline

Derived from existing source code in this repository.

---

## File Structure

Every file starts with the full MIT license header, followed by a `DESCRIPTION` Doxygen block, then fixed section separators in this order:

```cpp
/* MIT License
 *
 * Copyright (c) 2023 - 2026 Andreas Merkle <web@blue-andi.de>
 * ...
 */

/*******************************************************************************
    DESCRIPTION
*******************************************************************************/
/**
 * @file
 * @brief  Short description.
 * @author First Last <email@example.com>
 *
 * @addtogroup GroupName
 *
 * @{
 */
```

Section separators inside `.cpp` files appear in this fixed order:

```
Includes / Compiler Switches / Macros / Types and classes /
Prototypes / Local Variables / Public Methods / Protected Methods /
Private Methods / External Functions / Local Functions
```

Each section uses a 80-character comment banner:

```cpp
/******************************************************************************
 * Includes
 *****************************************************************************/
```

Close every header file with:

```cpp
#endif /* FILENAME_H */

/** @} */
```

---

## Naming

| Element | Convention | Example |
|---|---|---|
| Class | PascalCase | `StateMachine`, `StartupState` |
| Interface / abstract base | Prefix `I` | `IState`, `IBoard` |
| Method | camelCase | `setup()`, `getInstance()` |
| Private member variable | `m_` prefix + camelCase | `m_isActive`, `m_duration` |
| Constant / `static const` member | `UPPER_SNAKE_CASE` | `MIN_BATTERY_LEVEL`, `SEND_WAYPOINT_TIMER_INTERVAL` |
| File-scope static variable | `g` prefix + camelCase | `gLogSinkSerial` |
| Macro parameter | double-underscore prefix | `SIMPLE_TIMER_SECONDS(__timeInS)` |
| Enum value | `UPPER_SNAKE_CASE` | `CMD_GET_MAX_SPEED`, `CMD_NONE` |

Use `uint8_t`, `uint16_t`, `uint32_t` etc. for sized integer types. Append `U` to unsigned integer literals (`50U`, `1000U`).

---

## Headers

Use `#ifndef` include guards — not `#pragma once`:

```cpp
#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H
// ...
#endif /* STATE_MACHINE_H */
```

Template/header-only implementations use `.hpp`, all others use `.h` / `.cpp`.

Include order (within a file):
1. Own header (`"App.h"`)
2. Project headers (`<Board.h>`, `<Logging.h>`)
3. Third-party / Arduino headers (`<ArduinoJson.h>`)

---

## Doxygen

Every public method, class, and file gets a Doxygen doc comment. Use `/** */` style.

```cpp
/**
 * Set latest vehicle data received from RadonUlzer.
 *
 * @param[in] waypoint  Latest vehicle data.
 */
void setLatestVehicleData(const Waypoint& waypoint);

/**
 * Get pending command.
 *
 * @param[out] cmd  Pending command.
 *
 * @returns true if a command is pending, otherwise false.
 */
bool getPendingCommand(Command& cmd);
```

Use `@param[in]`, `@param[out]`, `@param[in,out]`, `@returns`. Document deleted copy operations too (mark as "Not allowed.").

---

## Class Layout

Ordering within a class: `public` → `protected` → `private`. Group within each section:

1. Constructors / destructor
2. Static factory / `getInstance()`
3. Public methods
4. Static constants
5. Member variables
6. Deleted copy constructor / assignment operator

---

## Patterns

### Singleton

```cpp
class StartupState : public IState
{
public:
    static StartupState& getInstance()
    {
        static StartupState instance;
        /* Singleton idiom to force initialization during first usage. */
        return instance;
    }

private:
    StartupState() : IState(), m_isActive(false) { }
    virtual ~StartupState() { }
    StartupState(const StartupState& state);             // Not allowed.
    StartupState& operator=(const StartupState& state);  // Not allowed.
};
```

### State Machine

States implement `IState` with three lifecycle methods:

```cpp
class IState
{
public:
    virtual void entry()                 = 0;
    virtual void process(StateMachine& sm) = 0;
    virtual void exit()                  = 0;
};
```

Concrete states mark the overrides `final` and are themselves singletons.

---

## Formatting

- **Indentation**: 4 spaces, no tabs.
- **Braces**: Allman style — opening brace on its own line for classes and functions; on the same line for `if`/`else`/`for`/`while`.
- **Constructor initializer lists**: one member per line, colon on the first line.

```cpp
App() :
    m_channelId(0U),
    m_timer(),
    m_stateMachine()
{
}
```

- **Variable alignment**: align `=` signs in blocks of related declarations.

```cpp
bool             isSuccessful = false;
SettingsHandler& settings     = SettingsHandler::getInstance();
Board&           board        = Board::getInstance();
```

---

## Conditionals

Write comparisons in "Yoda style" with explicit `true`/`false`:

```cpp
if (true == m_isActive)   { ... }
if (false == isSuccessful) { ... }
if (nullptr != pData)     { ... }
```

Use `nullptr` (never `NULL` or `0`) for pointer checks.

### Switch

Case labels are at the same indentation level as `switch`. Mark intentional fall-through explicitly:

```cpp
switch (status)
{
case STATUS_A:
    /* Fallthrough */
case STATUS_B:
    handleError();
    break;

default:
    break;
}
```

---

## Parameters

Pass non-trivial input arguments as `const` reference; output arguments as non-const reference:

```cpp
void process(const Waypoint& in, Result& out);
```

---

## Logging

Use severity macros — no direct `printf` in application code:

```cpp
LOG_FATAL("Setup failed.");
LOG_ERROR("Unexpected state %d.", state);
LOG_WARNING("Battery low.");
LOG_DEBUG("Speed: %d mm/s.", speed);
```

---

## Tests

Unit tests use the Unity framework (`TEST_ASSERT_*`, `RUN_TEST`). Each suite has `setUp()` and `tearDown()` functions. Test files live under `test/test_<Name>/`.
