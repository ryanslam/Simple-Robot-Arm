function dynamics_coupling_demo() {
    console.log("Client sending command DYNAMICS_COUPLING_DEMO")
    socket.emit("dynamics_coupling_demo")
}

function dynamics_time_varying_demo() {
    console.log("Client sending command DYNAMICS_TIME_VARYING_DEMO")
    socket.emit("dynamics_time_varying_demo")
}