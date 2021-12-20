import path from 'path';
import {spawn} from 'child_process';
import {TEMPERATURE} from '../src/components/Log/channels';
import msg, { LOG_ERROR, LOG_WARNING } from '../src/components/Log/LogItem';

const telemetry = async (win) => {
    let listener = spawn('python3', ['-u', path.resolve(__dirname, '../ros/src/telemetry/src/status.py')]);

    win.webContents.send(TEMPERATURE, msg(TEMPERATURE, 'Started Temperature node'));

    listener.on('exit', code => { 
        win.webContents.send(TEMPERATURE, msg(TEMPERATURE, 'Node exited', LOG_WARNING));
    });

    listener.stdout.on('data', data => {
        try{
            let temp = JSON.parse(data);
            win.webContents.send('temperature', temp);
        }catch(e){
            
        } 
    });

    listener.stderr.on('data', data => {
        win.webContents.send(TEMPERATURE, msg(TEMPERATURE, `Error: ${data}`, LOG_ERROR));
    });

    win.on('close', _ => {
        listener.kill('SIGINT');
    })
}

export default telemetry;