# component-monitoring
Package for managing monitors of components used in MigrAVE
TO-DO:
1. When component is registering, deregistering and registering for component-monitoring, it is able to run corresponding monitor only for the second trial.
2. Components should not ask `monitor_manager` to turn on/off the monitors but they rather should ask it to be registered/deregistered as a subscriber to certain monitors. `monitor_manager` should run the specific monitor if there is at least one subscriber and stop the monitor when there are no subscribers to it. 
3. Design decisions regarding `database_manager`. It should work similarly to `monitor_manager`.
4. Finish the knowledge_base component.
