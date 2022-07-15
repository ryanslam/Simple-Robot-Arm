function append_log_level(level) {
    var log_level = document.createElement("SMALL");
    log_level.innerHTML = level;
    
    switch(level){
        case "DEBUG":
            log_level.className = "log_window log_line debug";
            break;
        case "INFO":
            log_level.className = "log_window log_line info";
            break;
        case "WARNING":
            log_level.className = "log_window log_line warning";
            break;
        case "ERROR":
            log_level.className = "log_window log_line error";
            break;
        default:
            log_level.className = "log_window log_line info";
            console.log("Error setting log level class!");
    }
    document.getElementById("log_window").append(log_level)

}

socket.on("log", function (t, name, level, msg) {
    // recieved an output from one of the server loggers. add it to the telemetry window
    console.log("Received log message. Time: "+ t + " Level: " + level + " Msg: " + msg);
    var datetime = document.createElement("SMALL");
    var logger_name = document.createElement("SMALL");
    var msg_data = document.createElement("SMALL");
    var msg_break = document.createElement("SMALL");
    
    datetime.innerHTML = t;
    logger_name.innerHTML = name;
    msg_data.innerHTML = msg;

    // set the formatting of the elements
    datetime.className = "log_window log_line datetime";
    msg_data.className = "log_window log_line";

    var b = document.createElement("BR");

    // document.getElementById("log_window").prepend(b)        // necessary for the small items (not for p)
    // document.getElementById("log_window").prepend(msg_data)
    // append_log_level(level)
    // document.getElementById("log_window").prepend(logger_name)
    // document.getElementById("log_window").prepend(datetime)
    log_window = document.getElementById("log_window");
    log_window.append(b);        // necessary for the small items (not for p)
    log_window.append(datetime);
    log_window.append(logger_name);
    append_log_level(level);
    log_window.append(msg_data);

    shouldScroll = log_window.scrollTop + log_window.clientHeight === log_window.scrollHeight;
    if (!shouldScroll) {
        log_window.scrollTop = log_window.scrollHeight;
    }
});

socket.on("telemetry", function(motor_0_enabled, motor_1_enabled, current_controller, motor_0_gains, motor_1_gains, t1, t2, x, y){
    document.getElementById("motor_0_enabled").innerHTML = "Servo 0 enabled: " + motor_0_enabled + ";";
    document.getElementById("motor_1_enabled").innerHTML = "Servo 1 enabled: " + motor_1_enabled + ";";
    document.getElementById("current_controller").innerHTML = "Active controller: " + current_controller + ";";
    document.getElementById("controller_gains_0").innerHTML = "Controller gains motor 0 (PID)" + motor_0_gains + ";";
    document.getElementById("controller_gains_1").innerHTML = "Controller gains motor 1: (PID)" + motor_1_gains + ";";
    document.getElementById("ee_position").innerHTML = "X: " + String(x) + ", Y: " + String(y) + " (mm)";
    document.getElementById("joint_position").innerHTML = "Theta1: " + String(t1) + ", Theta2: " + String(t2) + " (deg)";
})