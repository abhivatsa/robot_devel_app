import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

import Stats from 'three/addons/libs/stats.module.js';

//-------------------- Websocket -----------------------
let socket = new WebSocket("ws://127.0.0.1:8080");
// var btn = document.getElementById("btn");
socket.onopen = function (e) {
    console.log("[open] Connection established");
    console.log("Sending to server");
    mychart.update();
};

socket.onmessage = function (event) {
    // console.log(`[message] Data received from server`);
    let incomint_data = JSON.parse(event.data);
    // console.log(incomint_data)
    updateRobotState(incomint_data.current_state.positions, incomint_data.current_state.velocities, incomint_data.current_state.torque, incomint_data.current_state.tcp_pose);
    updateSystemState(incomint_data.system_state);
};

socket.onclose = function (event) {
    if (event.wasClean) {
        console.log(`[close] Connection closed cleanly, code=${event.code} reason=${event.reason}`);
    } else {
        // e.g. server process killed or network down
        // event.code is usually 1006 in this case
        console.log('[close] Connection died');
    }
};

socket.onerror = function (error) {
    console.log(`[error]`);
};

// -----------------------------------------------------------




// ------------------------- ThreeJS ---------------------------
// const container = document.createElement( 'div' );
// document.body.appendChild( container );

const container = document.getElementById("render-container");

console.log(container);

let camera, scene, renderer, stats;

const clock = new THREE.Clock();
let mixer;

// camera = new THREE.PerspectiveCamera( 75, window.innerWidth / window.innerHeight, 0.1, 20 );
camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 1, 100);
camera.position.set(-2, 2, 2);

scene = new THREE.Scene();
scene.background = new THREE.Color(0xa0a0a0);
scene.fog = new THREE.Fog(0xa0a0a0, 2, 10);

const hemiLight = new THREE.HemisphereLight(0xffffff, 0x444444, 5);
hemiLight.position.set(0, 200, 0);
scene.add(hemiLight);

const dirLight = new THREE.DirectionalLight(0xffffff, 5);
dirLight.position.set(0, 200, 100);
dirLight.castShadow = true;
dirLight.shadow.camera.top = 180;
dirLight.shadow.camera.bottom = - 100;
dirLight.shadow.camera.left = - 120;
dirLight.shadow.camera.right = 120;
scene.add(dirLight);

const mesh = new THREE.Mesh(new THREE.PlaneGeometry(2000, 2000), new THREE.MeshPhongMaterial({ color: 0x999999, depthWrite: false }));
mesh.rotation.x = - Math.PI / 2;
mesh.receiveShadow = true;
scene.add(mesh);

const grid = new THREE.GridHelper(20, 40, 0x000000, 0x000000);
grid.material.opacity = 0.2;
grid.material.transparent = true;
scene.add(grid);

renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
// renderer.setSize(window.innerWidth, window.innerHeight);
renderer.setSize(container.innerWidth, container.innerHeight);
renderer.shadowMap.enabled = true;


container.appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 0.5, 0);
controls.update();

window.addEventListener('resize', onWindowResize);

// stats = new Stats();
// container.appendChild( stats.dom );

const loader1 = new GLTFLoader();
const loader2 = new GLTFLoader();
const loader3 = new GLTFLoader();
const loader4 = new GLTFLoader();
const loader5 = new GLTFLoader();
const loader6 = new GLTFLoader();
const loader7 = new GLTFLoader();

var link0;
var link1;
var link2;
var link3;
var link4;
var link5;
var link6;


loader1.load('./robot_gltf/link0.glb', function (gltf) {

    link0 = gltf.scene;
    scene.add(link0);

    loader2.load('./robot_gltf/link1.glb', function (gltf) {

        link1 = gltf.scene;
        link0.add(link1);
        link1.position.setY(0.075);

        loader3.load('./robot_gltf/link2.glb', function (gltf) {

            link2 = gltf.scene;
            link1.add(link2);
            link2.position.setY(0.124);
            link2.position.setZ(0.085);

            loader4.load('./robot_gltf/link3.glb', function (gltf) {

                link3 = gltf.scene;
                link2.add(link3);
                link3.position.setY(0.52);
                link3.position.setZ(0.0025)

                loader5.load('./robot_gltf/link4.glb', function (gltf) {

                    link4 = gltf.scene;
                    link3.add(link4);
                    link4.position.setY(0.118);
                    link4.position.setZ(-0.085);
                    // link4.rotation.y = -Math.PI/2;

                    loader6.load('./robot_gltf/link5.glb', function (gltf) {

                        link5 = gltf.scene;
                        link4.add(link5);
                        link5.position.setY(0.362);
                        link5.position.setZ(0.075);

                        loader7.load('./robot_gltf/link6.glb', function (gltf) {

                            link6 = gltf.scene;
                            link5.add(link6);
                            link6.position.setY(0.115);
                            link6.position.setZ(-0.072);
                        });

                    });

                });

            });

        });

    });
});

onWindowResize();

function onWindowResize() {

    // camera.aspect = window.innerWidth / window.innerHeight;
    camera.aspect  = 1.333;
    camera.updateProjectionMatrix();

    renderer.setSize(768, 576);

}



var y_val = 0;
function animate() {

    requestAnimationFrame(animate);

    const delta = clock.getDelta();

    if (mixer) mixer.update(delta);

    renderer.render(scene, camera);

    y_val = y_val + 0.01;
    
}

animate();
// -----------------------------------------------------------------------------------------------------

var current_time = 0;
var joint_positions = [0,0,0,0,0,0];
var joint_velocities = [0,0,0,0,0,0];
var joint_torque = [0,0,0,0,0,0];

var tcp_pose = [0,0,0,0,0,0];

// Update robot state
function updateRobotState(positions, velocities, torque, tcp)
{
    // console.log(positions);
    link1.rotation.y = positions.joint1;
    link2.rotation.z = positions.joint2;
    link3.rotation.z = positions.joint3;
    link4.rotation.y = positions.joint4;
    link5.rotation.z = positions.joint5;
    link6.rotation.y = positions.joint6;

    joint_val_1.innerText = positions.joint1.toFixed(2);
    joint_val_2.innerText = positions.joint2.toFixed(2);
    joint_val_3.innerText = positions.joint3.toFixed(2);
    joint_val_4.innerText = positions.joint4.toFixed(2);
    joint_val_5.innerText = positions.joint5.toFixed(2);
    joint_val_6.innerText = positions.joint6.toFixed(2);

    j1_slider.value = convertValToRange(positions.joint1, 0);
    j2_slider.value = convertValToRange(positions.joint2, 0);
    j3_slider.value = convertValToRange(positions.joint3, 0);
    j4_slider.value = convertValToRange(positions.joint4, 0);
    j5_slider.value = convertValToRange(positions.joint5, 0);
    j6_slider.value = convertValToRange(positions.joint6, 0);


    cart_val_x.innerHTML = (tcp.x*1000).toFixed(2); // mm
    cart_val_y.innerHTML = (tcp.y*1000).toFixed(2);
    cart_val_z.innerHTML = (tcp.z*1000).toFixed(2);
    cart_val_roll.innerHTML = tcp.roll.toFixed(2); //(tcp.roll*180/Math.PI).toFixed(2);
    cart_val_pitch.innerHTML = tcp.pitch.toFixed(2); //(tcp.pitch*180/Math.PI).toFixed(2);
    cart_val_yaw.innerHTML = tcp.yaw.toFixed(2); //(tcp.yaw*180/Math.PI).toFixed(2);

    x_slider.value = convertValToRange(tcp.x, 1);
    y_slider.value = convertValToRange(tcp.y, 1);
    z_slider.value = convertValToRange(tcp.z, 1);
    roll_slider.value = convertValToRange(tcp.roll, 0);
    pitch_slider.value = convertValToRange(tcp.pitch, 0);
    yaw_slider.value = convertValToRange(tcp.yaw, 0);

    current_time = current_time + 0.001;
    joint_positions[0] = positions.joint1;
    joint_positions[1] = positions.joint2;
    joint_positions[2] = positions.joint3;
    joint_positions[3] = positions.joint4;
    joint_positions[4] = positions.joint5;
    joint_positions[5] = positions.joint6;

    joint_velocities[0] = velocities.joint1;
    joint_velocities[1] = velocities.joint2;
    joint_velocities[2] = velocities.joint3;
    joint_velocities[3] = velocities.joint4;
    joint_velocities[4] = velocities.joint5;
    joint_velocities[5] = velocities.joint6;

    joint_torque[0] = torque.joint1;
    joint_torque[1] = torque.joint2;
    joint_torque[2] = torque.joint3;
    joint_torque[3] = torque.joint4;
    joint_torque[4] = torque.joint5;
    joint_torque[5] = torque.joint6;

    // updateChartData(positions);
}

function updateChartData()
{
    if(chart_x_data.length >= 10)
    {
        chart_x_data.shift();
        chart_position1.shift();
        chart_position2.shift();
        chart_position3.shift();
        chart_position4.shift();
        chart_position5.shift();
        chart_position6.shift();

        chart_velocity1.shift();
        chart_velocity2.shift();
        chart_velocity3.shift();
        chart_velocity4.shift();
        chart_velocity5.shift();
        chart_velocity6.shift();

        chart_torque1.shift();
        chart_torque2.shift();
        chart_torque3.shift();
        chart_torque4.shift();
        chart_torque5.shift();
        chart_torque6.shift();
    }

    chart_x_data.push(current_time.toFixed(2));
    chart_position1.push(joint_positions[0].toFixed(2));
    chart_position2.push(joint_positions[1].toFixed(2));
    chart_position3.push(joint_positions[2].toFixed(2));
    chart_position4.push(joint_positions[3].toFixed(2));
    chart_position5.push(joint_positions[4].toFixed(2));
    chart_position6.push(joint_positions[5].toFixed(2));

    chart_velocity1.push(joint_velocities[0].toFixed(2));
    chart_velocity2.push(joint_velocities[1].toFixed(2));
    chart_velocity3.push(joint_velocities[2].toFixed(2));
    chart_velocity4.push(joint_velocities[3].toFixed(2));
    chart_velocity5.push(joint_velocities[4].toFixed(2));
    chart_velocity6.push(joint_velocities[5].toFixed(2));

    chart_torque1.push(joint_torque[0].toFixed(2));
    chart_torque2.push(joint_torque[1].toFixed(2));
    chart_torque3.push(joint_torque[2].toFixed(2));
    chart_torque4.push(joint_torque[3].toFixed(2));
    chart_torque5.push(joint_torque[4].toFixed(2));
    chart_torque6.push(joint_torque[5].toFixed(2));

    mychart.data.labels = chart_x_data;
    if(show_chart_data == ShowChartData.Torque)
    {
        mychart.data.datasets[0].data = chart_torque1;
        mychart.data.datasets[1].data = chart_torque2;
        mychart.data.datasets[2].data = chart_torque3;
        mychart.data.datasets[3].data = chart_torque4;
        mychart.data.datasets[4].data = chart_torque5;
        mychart.data.datasets[5].data = chart_torque6;

        mychart.options.scales.yAxes[0].ticks.max = 10;
        mychart.options.scales.yAxes[0].ticks.min = -10;
        mychart.options.scales.yAxes[0].scaleLabel.labelString = "Joint torque (Nm)";
    }
    else if(show_chart_data == ShowChartData.Velocity)
    {
        mychart.data.datasets[0].data = chart_velocity1;
        mychart.data.datasets[1].data = chart_velocity2;
        mychart.data.datasets[2].data = chart_velocity3;
        mychart.data.datasets[3].data = chart_velocity4;
        mychart.data.datasets[4].data = chart_velocity5;
        mychart.data.datasets[5].data = chart_velocity6;

        mychart.options.scales.yAxes[0].ticks.max = 3.14;
        mychart.options.scales.yAxes[0].ticks.min = -3.14;
        mychart.options.scales.yAxes[0].scaleLabel.labelString = "Joint velocity (rad/s)";
    }
    else
    {
        mychart.data.datasets[0].data = chart_position1;
        mychart.data.datasets[1].data = chart_position2;
        mychart.data.datasets[2].data = chart_position3;
        mychart.data.datasets[3].data = chart_position4;
        mychart.data.datasets[4].data = chart_position5;
        mychart.data.datasets[5].data = chart_position6;

        mychart.options.scales.yAxes[0].ticks.max = 3.14;
        mychart.options.scales.yAxes[0].ticks.min = -3.14;
        mychart.options.scales.yAxes[0].scaleLabel.labelString = "Joint position (rad)";
    }
    mychart.update();
}

// Update system state
var prev_state = 0;
function updateSystemState(state) 
{
    if(prev_state == state.power_on_status)
    {
        return;
    }
    // update the data    
    if(state.power_on_status == 3) // power on
    {
        enableButtons(true);
        enablePowerBtn(true);
        powerBtn.checked = true;
        systemStateSpinner.classList.add("visually-hidden");
        systemStateText.innerHTML = "Ready";

        console.log("power on");
    }
    else if(state.power_on_status == 0) // power off
    {
        enableButtons(false);
        enablePowerBtn(true);

        systemStateSpinner.classList.add("visually-hidden");
        systemStateText.innerHTML = "Powered OFF";

        console.log("power off");
    }
    else if(state.power_on_status == 1) // initializing system
    {
        enableButtons(false);
        enablePowerBtn(false);

        systemStateSpinner.classList.remove("visually-hidden");
        systemStateText.innerHTML = "Initializing System...";

        console.log("Initializing System...");
    }
    else if(state.power_on_status == 2) // harware check
    {
        enableButtons(false);
        enablePowerBtn(false);

        systemStateSpinner.classList.remove("visually-hidden");
        systemStateText.textContent = "Hardware check...";

        console.log("Hardware check...");
    }
    else // In execution
    {
        enableButtons(false);
        enablePowerBtn(false);
        systemStateSpinner.classList.remove("visually-hidden");
        systemStateText.textContent = "Executing...";

        console.log("Executing...");
    }
    prev_state = state.power_on_status;
}

// Send json on Power button click
function powerBtnClicked()
{
    console.log("btn clicked");
    if(!powerBtn.checked)
    {
        enableButtons(false);
    }
    var cmd_obj =
    { 
        system_data : {
            power_on: powerBtn.checked,
            }
    };
    socket.send(JSON.stringify(cmd_obj));
    console.log(cmd_obj);
}

// Send json on Jog btn click
function onJogClicked(mode, index, dir)
{
    // send jog commands
    if(!checkState())
    {
        var m = 2;
        if(mode == "joint_space")
        {
             m = 0;
        }
        else if(mode == "task_space"){
            m = 1;
        } 
        var cmd_obj =
        { 
            command_data : {
                type: 1,
                mode: m,
                index: parseInt(index),
                direction: parseInt(dir)
                }
        };
        socket.send(JSON.stringify(cmd_obj));
    }
}

// Click and hold jog button callback
var t;
function onClickAndHold (mode, index, dir) {
    onJogClicked(mode, index, dir);
    t = setInterval(onJogClicked, 1000, mode, index, dir);
}


// Mode of operation change handle
function onModeChangeClicked(element)
{
    if(checkState(element))
    {
        element.checked = false;
    }
    if(checkState())
    {
        simulationMode.disabled = true;
        enablePowerBtn(false);
        
    }
    else
    {
        simulationMode.disabled = false;
        enablePowerBtn(true);
    }
}

function convertValToRange(val, type)
{
    if(type == 0) // radians
    {
        return val*800000/(2*Math.PI);
    }
    else
    {
        return val*800000/1.0;
    }
}

//------------------------DOM Manipulation-----------------------------------
// // Define event handler
const handler = e => {
    console.log(`Document is ready!`)
    
    // disable the buttons
    enableButtons(false);

    chart_container.style.display = "none";

    
}
  
// // Listen for `DOMContentLoaded` event
document.addEventListener('DOMContentLoaded', handler)

const input_control_tab = document.getElementById("inputControl");

// Input sliders
const teachMode = document.getElementById("teachMode");
const handControllerMode = document.getElementById("handController");
const simulationMode = document.getElementById("simulationMode");
const powerBtn = document.getElementById("powerOnMode");
const jogButtons = document.getElementsByClassName("jog");


var buttons = [teachMode, handControllerMode, simulationMode];

// joint values
var joint_val_1 = document.getElementById("joint_val_1");
var joint_val_2 = document.getElementById("joint_val_2");
var joint_val_3 = document.getElementById("joint_val_3");
var joint_val_4 = document.getElementById("joint_val_4");
var joint_val_5 = document.getElementById("joint_val_5");
var joint_val_6 = document.getElementById("joint_val_6");

// cartesian values
var cart_val_x = document.getElementById("cart_val_x");
var cart_val_y = document.getElementById("cart_val_y");
var cart_val_z = document.getElementById("cart_val_z");
var cart_val_roll = document.getElementById("cart_val_roll");
var cart_val_pitch = document.getElementById("cart_val_pitch");
var cart_val_yaw = document.getElementById("cart_val_yaw");

// joint sliders
var j1_slider = document.getElementById("slider_j1");
var j2_slider = document.getElementById("slider_j2");
var j3_slider = document.getElementById("slider_j3");
var j4_slider = document.getElementById("slider_j4");
var j5_slider = document.getElementById("slider_j5");
var j6_slider = document.getElementById("slider_j6");

// Cartesian Sliders
var x_slider = document.getElementById("slider_x");
var y_slider = document.getElementById("slider_y");
var z_slider = document.getElementById("slider_z");
var roll_slider = document.getElementById("slider_roll");
var pitch_slider = document.getElementById("slider_pitch");
var yaw_slider = document.getElementById("slider_yaw");


var graph_btn = document.getElementById("graphBtn");
var chart_container = document.getElementById("chartContainer");

var plot_position = document.getElementById("plot_position");
var plot_velocity = document.getElementById("plot_velocity");
var plot_torque = document.getElementById("plot_torque");

const systemStateText = document.getElementById("systemState");
const systemStateContainer = document.getElementById("systemStateContainer");
const systemStateSpinner = document.getElementById("systemStateSpinner");



// enable disble the buttons
function enableButtons(state) 
{
    teachMode.disabled = !state;
    handControllerMode.disabled = !state;
    simulationMode.disabled = !state;
    move_test.disabled = !state;

    for (let index = 0; index < jogButtons.length; index++)
    {
        jogButtons[index].disabled = !state;
    }

    if(state)
    {
        input_control_tab.classList.remove("disabled");
    }
    else
    {
        input_control_tab.classList.add("disabled")
    } 
}

// Check state of the buttons
function checkState(element)
{
    if(!powerBtn.checked)
    {
        return !powerBtn.checked;
    }
    else if(element == teachMode)
    {
        return handControllerMode.checked;
    }
    else if(element == handControllerMode)
    {
        return teachMode.checked;
    }
    else
    {
        return (teachMode.checked || handControllerMode.checked);
    }
}

// enable/disable power btn
function enablePowerBtn(state) {
    powerBtn.disabled = !state;
}

powerBtn.addEventListener('click', powerBtnClicked);

// jog Button event handler
for (let index = 0; index < jogButtons.length; index++) {
    const element = jogButtons[index];
    // element.onmousedown = ()=> {onClickAndHold(element.getAttribute("mode"),element.getAttribute("index"), element.getAttribute("direction"))};
    element.onmousedown = ()=> {onJogClicked(element.getAttribute("mode"),element.getAttribute("index"), element.getAttribute("direction"))};
    element.onmouseup = function () {
        onJogClicked("none",element.getAttribute("index"), element.getAttribute("direction"));
        clearTimeout(t);
    }
    element.onmouseleave = () => onJogClicked("none",element.getAttribute("index"), element.getAttribute("direction"));
}


// Mode button event handler
teachMode.onclick  = () => {onModeChangeClicked(teachMode)};
handControllerMode .onclick = () => {
    onModeChangeClicked(handControllerMode);

    var cmd_obj =
        { 
            command_data : {
                type: 2,
                }
        };
        socket.send(JSON.stringify(cmd_obj));



};


graph_btn.addEventListener("click", ()=>{
    console.log("clicked")
    if(chart_container.style.display == "none")
    {
       chart_container.style.display = "block";
       renderer.domElement.style.display = "none";
    }
    else
    {
        chart_container.style.display = "none";
        renderer.domElement.style.display = "block";
    }
})

//-----------------------------------------------------------------------


//--------------------------- TEST BUTTONS-----------------------------------------------------------
var move_test = document.getElementById("move_test");
move_test.onclick = ()=>{
    var cmd_obj =
        { 
            command_data : {
                type: 4,
                }
        };
        socket.send(JSON.stringify(cmd_obj));
}





//-------------------------------------- CHART JS --------------------
var chart_x_data = Array(10).fill(0);

var max_y_limit = 3.14;
var y_axis_label = "Joint Position (rad)";


var chart_position1 = Array(10).fill(0);
var chart_position2 = Array(10).fill(0);
var chart_position3 = Array(10).fill(0);
var chart_position4 = Array(10).fill(0);
var chart_position5 = Array(10).fill(0);
var chart_position6 = Array(10).fill(0);

var chart_velocity1 = Array(10).fill(0);
var chart_velocity2 = Array(10).fill(0);
var chart_velocity3 = Array(10).fill(0);
var chart_velocity4 = Array(10).fill(0);
var chart_velocity5 = Array(10).fill(0);
var chart_velocity6 = Array(10).fill(0);

var chart_torque1 = Array(10).fill(0);
var chart_torque2 = Array(10).fill(0);
var chart_torque3 = Array(10).fill(0);
var chart_torque4 = Array(10).fill(0);
var chart_torque5 = Array(10).fill(0);
var chart_torque6 = Array(10).fill(0);

const ShowChartData = {
    Position: 'position',
    Velocity: 'velocity',
    Torque: 'torque'
};

var show_chart_data = ShowChartData.Position;

var mychart = new Chart("myChart", {
    type: "line",
    data: {
        labels: Array(10).fill(0),
    datasets: [{
        data: Array(10).fill(0),
        borderColor: "red",
        fill: false,
        label: 'joint 1',
        radius: 1,
        title: "J1",
        },{
        data: Array(10).fill(0),
        borderColor: "green",
        fill: false,
        label: 'joint 2',
        radius: 1
        },{
        data: Array(10).fill(0),
        borderColor: "blue",
        fill: false,
        label: 'joint 3',
        radius: 1
        },{
        data: Array(10).fill(0),
        borderColor: "black",
        fill: false,
        label: 'joint 4',
        radius: 1
        },{
        data: Array(10).fill(0),
        borderColor: "violet",
        fill: false,
        label: 'joint 5',
        radius: 1
    },{
        data: Array(10).fill(0),
        borderColor: "tint",
        fill: false,
        label: 'joint 6',
        radius: 1
    }
]
    },
    options: {
        legend: {display: false},
        points:{radius: 0.1, borderWidth: 0.1},
        scales: {
        x: [{
            ticks: {
            callback: function(val) {
                return val.toFixed(2);
            }
            },
        }],
        // xAxes: [
        //     {scaleLabel: {
        //         display: true,
        //         labelString: 'time (s)'
        //       }}
        // ],
        yAxes : [{
            ticks : {
                max : max_y_limit,    
                min : -max_y_limit,
                stepSize: 0.1
            },
            scaleLabel: {
                display: true,
                labelString: y_axis_label
              }
        }]
        },
        animation: false,
        legend: {
                display: true,
                position : "bottom",
                maxSize : {
                    height : 50
                },
            labels : {
                usePointStyle : true
            }
        },
        // events: [] 
    }        
    });


    var updatechart = () =>{updateChartData();};
    var chart_ = setInterval(updatechart, 500);

    plot_position.onclick = ()=>{show_chart_data = ShowChartData.Position;}
    plot_velocity.onclick = ()=>{show_chart_data = ShowChartData.Velocity;}
    plot_torque.onclick = ()=>{show_chart_data = ShowChartData.Torque;}
//--------------------------------------------------------------------------------

