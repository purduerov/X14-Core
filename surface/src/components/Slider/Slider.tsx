import * as React from 'react';
import './Slider.scss';

//FULL SLIDER COMPONENT PROPS
interface Props{
    //vertical?: boolean
    min?: number
    max?: number
    step?: number
    unit?: string
    callback(val: number): void
    value: number
}

const defaultProps: Props = {
    //vertical: false,
    min: 0,
    max: 100,
    step: 1,
    callback: (_) => {},
    value: 0,
    unit: ''
}

//SLIDER BAR PROPS
interface sliderProps{
    min?: number
    max?: number
    step?: number
    callback(val: React.ChangeEvent): void
    value: number
}

//BOTTOM DISPLAY PROPS
interface displayProps{
    value: number
    unit? : string
    callback(val: boolean): void
}

//BOTTOM INPUT BOX PROPS
interface inputProps{
    value: number
    min?: number
    max?: number
    step?: number
    callback(val: React.FocusEvent): void
}

const SliderBar: React.FC<sliderProps> = ({value, min, max, step, callback}) => {
    return(
        <div className='slider-top'>
                <span className='slider-range'>{min}</span>
                <input 
                    type='range' 
                    min={min} 
                    max={max}
                    value={value}
                    step={step}
                    className='slider'
                    onChange={(e) => callback(e)}
                />
                <span className='slider-range'>{max}</span>
            </div>
    )
}

const ValueDisplay: React.FC<displayProps> = ({value, callback, unit}) => {
    return(
        <span style={{textAlign: 'center'}} onClick={() => {callback(true)}}>{value}{unit}</span>

    )
}

const Inputbox: React.FC<inputProps> = ({value, max, min, callback, step}) => {
    return(
        <input 
            autoFocus
            className='inputbox'
            type='number'
            min={min}
            max={max}
            defaultValue={value}
            step={step}
            onFocus={e => e.target.select()}
            onKeyUp={(e) => {
                if (e.key === 'Enter') {
                    const target = e.target as HTMLInputElement;
                    target.blur();
                }
            }}
            onBlur={(e) => callback(e)}
        ></input>
    )
}


const Slider: React.FC<Props> = (props) => {
    const [starting, setStarting] = React.useState(0);
    const [inputActive, setInputActive] = React.useState(false);

    React.useEffect(() => {
        setStarting(Object.assign({}, props).value);
    }, [])

    return (
        <div className='slider-container'>
            <SliderBar value={props.value} step={props.step} max={props.max} min={props.min} callback={(e) => {
                const target = e.target as HTMLInputElement
                props.callback(parseFloat(target.value))
            }}></SliderBar>
            <div className='slider-bottom'>
                <button className='zero-btn'
                    onClick={(e) => {
                        props.callback(starting)
                        const target = e.target as HTMLInputElement
                        target.blur()
                    }}>0</button>
                {inputActive ? (
                    <Inputbox value={props.value} max={props.max} min={props.min} step={props.step} callback={(e) => {
                        let target = e.target as HTMLInputElement
                        let changeto = parseFloat(target.value);
                        if(isNaN(changeto) || changeto === Infinity) {
                            changeto = props.value!;
                        }
                        if(changeto > props.max!) {
                            changeto = props.max!;
                        } else if(changeto < props.min!) {
                            changeto = props.min!;
                        }
                        props.callback(changeto);
                        setInputActive(false);
                    }}></Inputbox>
                ) : (
                    <ValueDisplay value={props.value} callback={(bool) => setInputActive(bool)}></ValueDisplay>
                )}
            </div>
        </div>
        
    )
}

Slider.defaultProps = defaultProps;

export default Slider;