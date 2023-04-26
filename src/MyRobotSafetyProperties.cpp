#include "MyRobotSafetyProperties.hpp"

MyRobotSafetyProperties::MyRobotSafetyProperties(ControlSystem &cs, double dt)
    : cs(cs),
    
      slSystemOff("System is offline"),
      slShuttingDown("System shutting down"),
      slBraking("System braking"),
      slStartingUp("System starting up"),
      slEmergency("Emergency"),
      slEmergencyBraking("System halting"),
      slSystemOn("System is online"),
      slMotorPowerOn("Motors powered"),
      slSystemMoving("System moving"),

      abort("Abort"),
      shutdown("Shutdown"),
      doSystemOn("Do system on"),
      systemStarted("System started"),
      emergency("Emergency"),
      powerOn("Power on"),
      powerOff("Power off"),
      startMoving("Start moving"),
      stopMoving("Stop moving"),
      resetEmergency("reset Emergency"),
      motorsHalted("Motors halted")
      
{
    eeros::hal::HAL &hal = eeros::hal::HAL::instance();

    // Declare and add critical outputs
    greenLED = hal.getLogicOutput("onBoardLEDgreen");
    redLED = hal.getLogicOutput("onBoardLEDred");

    criticalOutputs = {redLED, greenLED};

    // Declare and add critical inputs
    buttonMode = eeros::hal::HAL::instance().getLogicInput("onBoardButtonMode", true);
    buttonPause = eeros::hal::HAL::instance().getLogicInput("onBoardButtonPause", true);

    criticalInputs = {buttonMode, buttonPause};

    // Add all safety levels to the safety system (Defined in prior for the System)
    // Order: from lovest to the highest
    addLevel(slSystemOff);         // Level 0
    addLevel(slShuttingDown);      // Level 1
    addLevel(slBraking);           // Level 2
    addLevel(slStartingUp);        // Level 3
    addLevel(slEmergency);         // Level 4
    addLevel(slEmergencyBraking);  // Level 5
    addLevel(slSystemOn);          // Level 6
    addLevel(slMotorPowerOn);      // Level 7
    addLevel(slSystemMoving);      // Level 8

    // Add events to individual safety levels (Defined in prior for the System)
    slSystemOff.addEvent(doSystemOn, slStartingUp, kPublicEvent);
    slShuttingDown.addEvent(shutdown, slSystemOff, kPublicEvent);
    slBraking.addEvent(motorsHalted, slShuttingDown, kPublicEvent);
    slStartingUp.addEvent(systemStarted, slSystemOn, kPublicEvent);
    slEmergency.addEvent(resetEmergency, slSystemOn, kPublicEvent);
    slEmergencyBraking.addEvent(motorsHalted, slEmergency, kPublicEvent);
    slSystemOn.addEvent(powerOn, slMotorPowerOn, kPublicEvent);
    slMotorPowerOn.addEvent(powerOff, slSystemOn, kPublicEvent);
    slMotorPowerOn.addEvent(startMoving, slSystemMoving, kPublicEvent);
    slSystemMoving.addEvent(stopMoving, slMotorPowerOn, kPublicEvent);
    slSystemMoving.addEvent(emergency, slEmergency, kPublicEvent);
    slSystemMoving.addEvent(abort, slBraking, kPublicEvent);

    // Add events to multiple safety levels
    // addEventToAllLevelsBetween(lowerLevel, upperLevel, event, targetLevel, kPublicEvent/kPrivateEvent);
    
    // Define input actions for all levels
    slSystemOff.setInputActions({           ignore(buttonPause),                    ignore(buttonMode) });
    slShuttingDown.setInputActions({        ignore(buttonPause),                    ignore(buttonMode) });
    slBraking.setInputActions({             ignore(buttonPause),                    ignore(buttonMode) });
    slStartingUp.setInputActions({          ignore(buttonPause),                    ignore(buttonMode) });
    slEmergency.setInputActions({           ignore(buttonPause),                    check(buttonMode, false, resetEmergency) });
    slEmergencyBraking.setInputActions({    ignore(buttonPause),                    ignore(buttonMode) });
    slSystemOn.setInputActions({            check(buttonPause, false, emergency),   ignore(buttonMode) });
    slMotorPowerOn.setInputActions({        check(buttonPause, false, emergency),   ignore(buttonMode) });
    slSystemMoving.setInputActions({        check(buttonPause, false, emergency),   ignore(buttonMode) });

    // Define output actions for all levels
    slSystemOff.setOutputActions({           set(greenLED, false),   set(redLED, false) });
    slShuttingDown.setOutputActions({        set(greenLED, false),   set(redLED, true) });
    slBraking.setOutputActions({             set(greenLED, false),   set(redLED, true) });
    slStartingUp.setOutputActions({          set(greenLED, true),    set(redLED, false) });
    slEmergency.setOutputActions({           set(greenLED, true),    set(redLED, true) });
    slEmergencyBraking.setOutputActions({    set(greenLED, true),    set(redLED, true) });
    slSystemOn.setOutputActions({            set(greenLED, true),    set(redLED, false) });
    slMotorPowerOn.setOutputActions({        set(greenLED, true),    set(redLED, false) });
    slSystemMoving.setOutputActions({        set(greenLED, true),    set(redLED, false) });

    // Define and add level actions
    slSystemOff.setLevelAction([&](SafetyContext *privateContext) {
        eeros::Executor::stop();
    });

    slShuttingDown.setLevelAction([&](SafetyContext *privateContext) {
        cs.timedomain.stop();
        privateContext->triggerEvent(shutdown);
    });

    slBraking.setLevelAction([&](SafetyContext *privateContext) {
        // Check if motors are standing sill
        privateContext->triggerEvent(motorsHalted);
    });

    slStartingUp.setLevelAction([&](SafetyContext *privateContext) {
        cs.timedomain.start();
        privateContext->triggerEvent(systemStarted);
    });

    slEmergency.setLevelAction([&](SafetyContext *privateContext) {
        
    });

    slEmergencyBraking.setLevelAction([&](SafetyContext *privateContext) {
        // Check if motors are standing still
        privateContext->triggerEvent(motorsHalted);
    });

    slSystemOn.setLevelAction([&, dt](SafetyContext *privateContext) {
        if (slSystemOn.getNofActivations()*dt >= 1)   // wait 1 sec
        {
            privateContext->triggerEvent(powerOn);
        }
    });

    slMotorPowerOn.setLevelAction([&, dt](SafetyContext *privateContext) {
        if (slMotorPowerOn.getNofActivations()*dt >= 5)   // wait 5 sec
        {
            privateContext->triggerEvent(startMoving);
        }
    });

    slSystemMoving.setLevelAction([&, dt](SafetyContext *privateContext) {
        if (slSystemMoving.getNofActivations()*dt >= 5)   // wait 5 sec
        {
            privateContext->triggerEvent(stopMoving);
        }
    });

    // Define entry level (lowest safety level)
    setEntryLevel(slSystemOff);

    // Define exit function (terminates the program safely)
    exitFunction = ([&](SafetyContext *privateContext) {
        privateContext->triggerEvent(abort);
    });
}