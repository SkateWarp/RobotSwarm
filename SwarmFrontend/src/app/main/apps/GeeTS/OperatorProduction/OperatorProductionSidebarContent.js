import { motion } from "framer-motion";
import { useDispatch } from "react-redux";
import { useEffect, useState } from "react";
import * as moment from "moment";
import { Controller, useForm } from "react-hook-form";
import { Button, DialogContent, InputLabel, MenuItem, Paper, Select, TextField } from "@mui/material";
import Typography from "@mui/material/Typography";
import FormControl from "@mui/material/FormControl";
import { URL } from "app/constants/constants";
import axios from "axios";
import { getOperatorProductions } from "./store/operatorProductionSlice";

const date = new Date();
const firstDayCurrentMonth = new Date(date.getFullYear(), date.getMonth(), 1);

const defaultValues = {
    startDate: moment(firstDayCurrentMonth).format("YYYY-MM-DD[T]06:00"),
    endDate: moment().format("YYYY-MM-DD[T]23:59"),
    operatorId: 0,
};

function OperatorProductionSidebarContent() {
    const dispatch = useDispatch();
    const [operators, setOperators] = useState([]);

    const { control, watch, handleSubmit } = useForm({
        mode: "onChange",
        defaultValues,
    });

    const startDate = watch("startDate");
    const endDate = watch("endDate");
    const operatorId = watch("operatorId");

    useEffect(() => {
        axios.get(`${URL}/api/Accounts`).then((res) => {
            setOperators(res.data.data);
        });

        dispatch(
            getOperatorProductions({
                startDate,
                endDate,
                operatorId: 0,
                pageNumber: 1,
                pageSize: 10,
            })
        );

        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, []);

    function onSubmit() {
        dispatch(
            getOperatorProductions({
                startDate,
                endDate,
                operatorId,
                pageNumber: 1,
                pageSize: 10,
            })
        );
    }

    return (
        <div className="p-0 lg:p-24 lg:ltr:pr-4 lg:rtl:pl-4">
            <Paper
                component={motion.div}
                initial={{ y: 20, opacity: 0 }}
                animate={{ y: 0, opacity: 1, transition: { delay: 0.2 } }}
                className="rounded-0 shadow-none lg:rounded-16 lg:shadow-1"
            >
                <form
                    noValidate
                    onSubmit={handleSubmit(onSubmit)}
                    className="flex flex-col md:overflow-hidden"
                >
                    <DialogContent classes={{ root: "p-24" }}>
                        <Typography variant="subtitle1">Fecha de inicio</Typography>
                        <Controller
                            control={control}
                            name="startDate"
                            render={({ field }) => (
                                <TextField
                                    {...field}
                                    className="mb-32"
                                    variant="outlined"
                                    type="datetime-local"
                                    required
                                    fullWidth
                                />
                            )}
                        />

                        <Typography variant="subtitle1">Fecha de fin</Typography>
                        <Controller
                            control={control}
                            name="endDate"
                            render={({ field }) => (
                                <TextField
                                    {...field}
                                    className="mb-32"
                                    variant="outlined"
                                    type="datetime-local"
                                    required
                                    fullWidth
                                />
                            )}
                        />

                        <Controller
                            name="operatorId"
                            control={control}
                            render={({ field }) => (
                                <FormControl className="flex w-full sm:w-170 mx-2 mb-32" variant="outlined">
                                    <InputLabel htmlFor="category-label-placeholder">Operador</InputLabel>
                                    <Select {...field} label="Operator">
                                        <MenuItem value={0} key={0}>
                                            Todos
                                        </MenuItem>
                                        {operators.map((operator) => (
                                            <MenuItem value={operator.id} key={operator.id}>
                                                {operator.firstName} {operator.lastName}
                                            </MenuItem>
                                        ))}
                                    </Select>
                                </FormControl>
                            )}
                        />

                        <div className="flex w-full flex-col">
                            <Button variant="contained" color="primary" type="submit">
                                OK
                            </Button>
                        </div>
                    </DialogContent>
                </form>
            </Paper>
        </div>
    );
}

export default OperatorProductionSidebarContent;
