function controls_refresher_demo() {
    console.log("Client sending command CONTROLS_REFRESHER_DEMO")
    socket.emit("controls_refresher")
}

function workspace_demo() {
    console.log("Client sending command WORKSPACE_DEMO")
    theta_1_min = document.getElementById("theta_1_min_workspace").value
    theta_1_max = document.getElementById("theta_1_max_workspace").value
    if(parseFloat(theta_1_max) > 90 || parseFloat(theta_1_max) < -90 || parseFloat(theta_1_min) > 90 || parseFloat(theta_1_min) < -90){
        document.getElementById("guessed_solution").innerHTML = "Error, the max and minimum values lie within the range of (-90,90)";
    }
    else{
        document.getElementById("guessed_solution").innerHTML = "";
        socket.emit("workspace_demo", theta_1_min, theta_1_max);
        
    }
}