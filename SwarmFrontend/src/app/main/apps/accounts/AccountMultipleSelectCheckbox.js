import PropTypes from "prop-types";
import {Checkbox, FormControl, InputLabel, ListItemText, MenuItem, OutlinedInput, Select} from "@mui/material";

const ITEM_HEIGHT = 48;
const ITEM_PADDING_TOP = 8;

const MenuProps = {
    PaperProps: {
        style: {
            maxHeight: ITEM_HEIGHT * 4.5 + ITEM_PADDING_TOP,
            width: 250,
        },
    },
};

export default function AccountMultipleSelectCheckbox({dataList, selectedValue, setValue, notificationType}) {

    const handleChange = (event) => {
        setValue(event.target.value);
    };


    return (

        <div className="flex flex-col w-full mb-24">
            <FormControl sx={{
                '& .MuiTextField-root': {m:1, width: '43ch'},
            }}>
                <InputLabel id="multiple-checkbox-label">{notificationType}</InputLabel>
                <Select
                    labelId="multiple-checkbox-label"
                    id="multiple-checkbox"
                    multiple
                    value={selectedValue}
                    onChange={handleChange}
                    input={<OutlinedInput label={notificationType} />}
                    renderValue={(selected) => selected.join(', ')}
                    MenuProps={MenuProps}
                >
                    {dataList.map((data) => (

                        <MenuItem key={data.value} value={data.label} >
                            <Checkbox checked={selectedValue.indexOf(data.label) > -1} />
                            <ListItemText primary={data.label} />
                        </MenuItem>
                    ))}
                </Select>
            </FormControl>
        </div>
    );
}

AccountMultipleSelectCheckbox.propTypes = {

    dataList: PropTypes.array.isRequired,
    selectedValue: PropTypes.array.isRequired,
    setValue: PropTypes.func.isRequired,
    notificationType: PropTypes.string.isRequired
};