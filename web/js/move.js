
var speed = 20;
var web_control = true;
var twist = new ROSLIB.Message({
    linear : {
    x : 0.0,
    y : 0.0,
    z : 0.0
    },
    angular : {
    x : 0.0,
    y : 0.0,
    z : 0.0
    }
});

function range_change(data){
    // change speed`
    document.getElementById("speed_enter").value = data;
    speed = data
}

function enter_change(data){
    // change speed
    document.getElementById("speed_range").value = data;
    speed = data
}

function control_type(data){
    // button or keybord
    if(data==0){
        document.getElementById("button_enable").style = "width:120px;background-color: #d1d1d1";
        document.getElementById("keybord_enable").style = "width:120px;background-color: #ffffff";
        KeyboardState(false);
        web_control = true;
        console.log("button control");
    }else if(data==1){
        document.getElementById("button_enable").style = "width:120px;background-color: #ffffff";
        document.getElementById("keybord_enable").style = "width:120px;background-color: #d1d1d1";
        KeyboardState(true);
        web_control = false;
        console.log("keyboard control");
    }
}

function movement(data){
    if (web_control == true){
        var x_speed = 0;
        var y_speed = 0;
        var z_speed = 0;
        if(data==1){
            //forward
            x_speed = 1;
            y_speed = 0;
            z_speed = 0;
        }else if(data==2){
            //back
            x_speed = -1;
            y_speed = 0;
            z_speed = 0;
        }else if(data==3){
            //shift left
            x_speed = 0;
            y_speed = 1;
            z_speed = 0;
        }else if(data==4){
            // shift right
            x_speed = 0;
            y_speed = -1;
            z_speed = 0;
        }else if(data==5){
            // turn left
            x_speed = 0;
            y_speed = 0;
            z_speed = 1;
        }else if(data==6){
            //turn right
            x_speed = 0;
            y_speed = 0;
            z_speed = -1;
        }else{
            //stop
            x_speed = 0;
            y_speed = 0;
            z_speed = 0;
        }

        twist = new ROSLIB.Message({
            linear : {
            x : 0.05*x_speed*speed,
            y : 0.05*y_speed*speed,
            z : 0.0
            },
            angular : {
            x : 0.0,
            y : 0.0,
            z : 0.05*z_speed*speed
            }
        });
        pub_cmdVel.publish(twist)
    }
}
