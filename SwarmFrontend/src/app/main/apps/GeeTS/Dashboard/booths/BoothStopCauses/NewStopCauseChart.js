import { useEffect, useRef } from "react";
import * as echarts from "echarts";
import { useSelector } from "react-redux";

function NewStopCauseChart() {
    const myChart = useRef(null);
    const report = useSelector(
        ({ boothDashboardDetailsApp }) => boothDashboardDetailsApp.boothDashboards.report
    );

    // let rAccumulatedTime = 0;
    // const [accumulatedTime, setAccumulatedTime] = useState(null);
    // const accLabel = report ? `Tiempo acumulado: ${formatTime(accumulatedTime)}` : "";

    function getSeconds(name, info) {
        for (let i = 0; i < info.length; i++) {
            if (name === info[i].name) {
                return info[i].seconds;
            }
        }
    }

    function handleChartChange(params, info) {
        const { selected } = params;
        let count = 0;

        for (const name in selected) {
            if (!selected[name]) {
                const elementSec = getSeconds(name, info);
                count += elementSec;
            }
        }
        // setAccumulatedTime(rAccumulatedTime - count);
    }

    useEffect(() => {
        const chart = myChart !== null ? echarts.init(myChart.current) : myChart;
        const info = [];

        if (report) {
            // eslint-disable-next-line func-names
            report.alarmChartResponse.forEach(function (alarm) {
                info.push({
                    value: (Number(alarm.timePercentage) * 100).toFixed(2),
                    name: alarm.alarmReason.description,
                    seconds: alarm.seconds,
                    // times
                });
            });
            // rAccumulatedTime = report.accumulatedTime;
            // setAccumulatedTime(rAccumulatedTime);
        }

        chart.setOption({
            legend: {
                show: true,
                type: "scroll",
                orient: "horizontal",
                left: 0,
                top: "bottom",
            },
            tooltip: {
                trigger: "item",
                formatter(params) {
                    const colorSpan = (color) =>
                        `<span style='display:inline-block;margin-right:5px;border-radius:10px;width:9px;height:9px;background-color:${color}'></span>`;
                    let rez = "<span> </span>";

                    let xx = "";

                    if (params.data)
                        xx = `${colorSpan(params.color)} ${params.data.name}: ${params.data.value}%`;

                    rez += xx;

                    return rez;
                },
            },
            grid: {
                left: "30%",
                top: 0,
                right: "2%",
                bottom: 0,
            },
            graphic: [
                {
                    type: "text",
                    top: 0,
                    z: 100,
                    style: {
                        text: report ? `Tiempo total: ${formatTime(`${report && report.totalTime}`)}` : "",
                        font: "bold 16px Microsoft YaHei",
                    },
                },
            ],
            series: {
                type: "pie",
                data: info,
                center: ["50%", "50%"],
                animationType: "scale",
                label: {
                    overflow: "break",
                    width: 110,
                },
            },
            textStyle: {
                color: "#000000",
            },
        });
        chart.on("legendselectchanged", (params) => handleChartChange(params, info));
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, [report]);

    function formatTime(seconds) {
        // eslint-disable-next-line radix
        return [parseInt(seconds / 60 / 60), parseInt((seconds / 60) % 60), parseInt(seconds % 60)]
            .join(":")
            .replace(/\b(\d)\b/g, "0$1");
    }

    return (
        <>
            {/* Todo verificar si el tiempo acumulado sera necesario en este grafico */}
            {/* <div */}
            {/*    className="font-bold" */}
            {/*    style={{ */}
            {/*        fontSize: "16px", */}
            {/*        fontFamily: "Microsoft YaHei", */}
            {/*        position: "absolute", */}
            {/*        marginTop: "40px", */}
            {/*    }} */}
            {/* > */}
            {/*    {accLabel} */}
            {/* </div> */}
            <div
                ref={myChart}
                style={{
                    width: "100%",
                    height: "500px",
                    marginTop: "20px",
                }}
            />
        </>
    );
}

export default NewStopCauseChart;
