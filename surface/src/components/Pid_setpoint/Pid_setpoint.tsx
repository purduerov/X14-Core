import { ipcRenderer } from 'electron';
import * as React from 'react';
import Slider from '../Slider/Slider';
import './Pid_setpoint.scss';

const names = ['Setpoint'];

//sets the initial setpoint to 0, will eventually need to set the setpoint to current depth
const Pid_setpoint: React.FC = () => {
    const [setpoint, setValues] = React.useState<Array<number>>([0.0])

    return(
        <div className='pid-container'>
            <div className='pid-title'>Depth Pid Plant</div>
            {setpoint.map((val, idx) => {
                return(
                    <Slider
                        value={val}
                        key={idx}
                        min={-1.0}
                        max={10.0}
                        step={0.1}
                        callback={(val) => {
                            let temp = [...setpoint];
                            temp[idx] = val;
                            setValues(temp);

                            ipcRenderer.send('depth_setpoint', temp);
                        }}
                    /> 
                )
            })}
        </div>
    )
}

export default Pid_setpoint;