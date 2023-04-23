import PropTypes from "prop-types";
import {useEffect, useRef} from "react";
import * as echarts from 'echarts';

const format = function (value) {
    return `${value}%`;
};

export default function Gauge({
                                  splitNumber,
                                  color,
                                  data,
                                  titleFontSize = 20,
                                  detailFontSize = 50,
                                  height = '300px',
                                  lineWidth = 6,
                                  axisLength = 12,
                                  splitLine = 20,
                                  letterColor = '#464646',
                                  center = '70%'
                              }) {
    const myChart = useRef(null);

    useEffect(() => {
        const chart = myChart !== null ? echarts.init(myChart.current) : myChart;

        chart.setOption({
            grid: {
                height: 10
            },

            series: [
                {
                    type: 'gauge',
                    startAngle: 180,
                    endAngle: 0,
                    min: 0,
                    max: 100,
                    splitNumber,
                    axisLine: {
                        lineStyle: {
                            width: lineWidth,
                            color
                        }
                    },
                    pointer: {
                        itemStyle: {
                            color: 'inherit'
                        }
                    },
                    axisTick: {
                        length: axisLength,
                        lineStyle: {
                            color: 'auto',
                            width: 2
                        }
                    },
                    splitLine: {
                        length: splitLine,
                        lineStyle: {
                            color: 'auto',
                            width: 5
                        }
                    },
                    axisLabel: {
                        show: false
                    },
                    title: {
                        fontSize: titleFontSize,
                        offsetCenter: [0, center],
                        color: letterColor,
                        fontWeight: 'bold'
                    },
                    detail: {
                        fontSize: detailFontSize,
                        // offsetCenter: [0, '0%'],
                        valueAnimation: true,
                        formatter: format,
                        color: 'inherit'
                    },
                    data,
                    animationDelay: 500
                }
            ]
        });
    }, []);

    return (
        <div
            ref={myChart}
            style={{
                width: '100%',
                height
            }}
        />
    );
}

Gauge.propTypes = {
    splitNumber: PropTypes.number.isRequired,
    color: PropTypes.array.isRequired,
    // formatter: PropTypes.func,
    data: PropTypes.arrayOf(Object).isRequired,
    titleFontSize: PropTypes.number,
    detailFontSize: PropTypes.number,
    height: PropTypes.string,
    lineWidth: PropTypes.number,
    axisLength: PropTypes.number,
    splitLine: PropTypes.number,
    letterColor: PropTypes.string
};
