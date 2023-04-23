import { yupResolver } from "@hookform/resolvers/yup";
import { Controller, useForm } from "react-hook-form";
import _ from "@lodash";
import Avatar from "@mui/material/Avatar";
import Button from "@mui/material/Button";
import TextField from "@mui/material/TextField";
import * as yup from "yup";

const schema = yup.object().shape({
    message: yup.string().required("Debe escribir comentario"),
});

function CardComment(props) {
    const defaultValues = {
        idMember: "-1",
        message: "",
    };
    const { control, formState, handleSubmit } = useForm({
        mode: "onChange",
        defaultValues,
        resolver: yupResolver(schema),
    });

    const { isValid, dirtyFields, errors } = formState;

    return (
        <form onSubmit={handleSubmit(props.onSubmit)} className="flex -mx-8 ">
            <Avatar className="w-32 h-32 mx-8" alt="Nombre" />
            <div className="flex flex-col items-start flex-1 mx-8 w-400 h-1/2">
                <Controller
                    name="message"
                    control={control}
                    render={({ field }) => (
                        <TextField
                            {...field}
                            className="flex flex-1"
                            multiline
                            fullWidth
                            error={!!errors.message}
                            helperText={errors?.message?.message}
                            row={3}
                            variant="outlined"
                            label="Escribe comentario"
                        />
                    )}
                />
                <Button
                    className="mt-16 mb-16"
                    aria-label="save"
                    variant="contained"
                    color="secondary"
                    type="submit"
                    size="small"
                    disabled={_.isEmpty(dirtyFields) || !isValid}
                >
                    Guardar
                </Button>
            </div>
        </form>
    );
}

export default CardComment;
