import path from 'path';
import {spawn} from 'child_process';
import msg, { LOG_ERROR, LOG_SUCCESS, LOG_WARNING } from '../src/components/Log/LogItem';
import {PID_SETPOINT} from '../src/components/Log/channels';
import net from 'net';
import {ipcMain} from 'electron';
import Pid_setpoint from 'src/components/Pid_setpoint/Pid_setpoint';

const messager = (win) => {
    let sock = net.connect(11002, undefined, () => {
        win.webContents.send(PID_SETPOINT, msg('pid_setpoint', 'Socket connected', LOG_SUCCESS));
    });

    ipcMain.on('depth_setpoint', (e, data: number,) => {
        sock.write(`${data}`);
    })

    sock.on('error', _ => win.webContents.send(PID_SETPOINT, msg('pid_setpoint', 'Socket error!', LOG_ERROR)));

    win.on('close', sock.end);
}

const depth_setpoint = async (win) => {
    let sender = spawn('python3', ['-u', path.resolve(__dirname, '../ros/src/pid_setpoint/src/pid_setpoint.py'), '11002']);
    
    win.webContents.send(PID_SETPOINT, msg('pid_setpoint', 'Started'));

    sender.on('exit', code => { 
        win.webContents.send(PID_SETPOINT, msg('pid_setpoint', 'Exiting...', LOG_WARNING));
    });

    sender.stdout.on('data', data => {
        if(`${data}`.includes('ready')){
            messager(win);
        }
    })

    sender.stderr.on('data', data => {
        win.webContents.send(PID_SETPOINT, msg('pid_setpoint', `Error: ${data}`, LOG_ERROR));
    })

    win.on('close', _ => {
        sender.kill('SIGINT');
        sender.kill('SIGINT');
    })
}

export default depth_setpoint;