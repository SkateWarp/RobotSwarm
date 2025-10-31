import { useEffect, useRef } from "react";
import PropTypes from "prop-types";
import * as echarts from 'echarts';
import { Card, CardContent, Typography } from "@mui/material";

function SensorChart({ sensorName, data }) {
    const chartRef = useRef(null);
    const chartInstance = useRef(null);

    useEffect(() => {
        if (!chartRef.current || !data || data.length === 0) return;

        // Initialize chart if not already initialized
        if (!chartInstance.current) {
            chartInstance.current = echarts.init(chartRef.current);
        }

        // Extract time and value arrays from data
        const times = data.map(d => d.time);
        const values = data.map(d => d.value);

        const option = {
            grid: {
                left: '10%',
                right: '5%',
                bottom: '15%',
                top: '10%',
            },
            xAxis: {
                type: 'category',
                data: times,
                axisLabel: {
                    rotate: 45,
                    fontSize: 10
                }
            },
            yAxis: {
                type: 'value',
                scale: true,
            },
            tooltip: {
                trigger: 'axis',
                formatter: (params) => {
                    const param = params[0];
                    return `${param.name}<br/>${param.seriesName}: ${param.value.toFixed(2)}`;
                }
            },
            series: [
                {
                    name: 'Valor',
                    data: values,
                    type: 'line',
                    smooth: true,
                    lineStyle: {
                        color: '#1976d2',
                        width: 2
                    },
                    itemStyle: {
                        color: '#1976d2'
                    },
                    areaStyle: {
                        color: new echarts.graphic.LinearGradient(0, 0, 0, 1, [
                            { offset: 0, color: 'rgba(25, 118, 210, 0.3)' },
                            { offset: 1, color: 'rgba(25, 118, 210, 0.05)' }
                        ])
                    }
                }
            ]
        };

        chartInstance.current.setOption(option);

        // Handle resize
        const handleResize = () => {
            chartInstance.current?.resize();
        };
        window.addEventListener('resize', handleResize);

        return () => {
            window.removeEventListener('resize', handleResize);
        };
    }, [data, sensorName]);

    // Cleanup on unmount
    useEffect(() => {
        return () => {
            chartInstance.current?.dispose();
        };
    }, []);

    if (!data || data.length === 0) return null;

    return (
        <Card className="mb-16">
            <CardContent>
                <Typography variant="h6" className="mb-12" sx={{ textTransform: 'uppercase', fontSize: '1rem' }}>
                    {sensorName.replace(/_/g, ' ')}
                </Typography>
                <div ref={chartRef} style={{ width: '100%', height: '200px' }} />
            </CardContent>
        </Card>
    );
}

SensorChart.propTypes = {
    sensorName: PropTypes.string.isRequired,
    data: PropTypes.arrayOf(PropTypes.shape({
        time: PropTypes.string.isRequired,
        value: PropTypes.number.isRequired,
    })).isRequired,
};

export default SensorChart;
